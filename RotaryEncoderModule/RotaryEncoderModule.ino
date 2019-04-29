/*
  ----------------------------------------------------------------------------

  This file is part of the Sanworks Bpod_Gen2 repository
  Copyright (C) 2017 Sanworks LLC, Stony Brook, New York, USA

  ----------------------------------------------------------------------------

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, version 3.

  This program is distributed  WITHOUT ANY WARRANTY and without even the
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

// The rotary encoder module, powered by Teensy 3.5, interfaces Bpod with a 1024-position rotary encoder: Yumo E6B2-CWZ3E
// The serial interface allows the user to set position thresholds, which generate Bpod events when crossed.
// The MATLAB interface can then retrieve the last trial's position log, and set new thresholds.
// MATLAB can also stream the current position to a plot, for diagnostics.

#include "ArCOM.h"
#include <SPI.h>
#include "SdFat.h"
#define FirmwareVersion 5
SdFatSdioEX SD;
ArCOM myUSB(SerialUSB); // USB is an ArCOM object. ArCOM wraps Arduino's SerialUSB interface, to
ArCOM StateMachineCOM(Serial3); // UART serial port
ArCOM OutputStreamCOM(Serial2); // UART serial port
// simplify moving data types between Arduino and MATLAB/GNU Octave.
File DataFile; // File on microSD card, to store position data

// Module setup
char moduleName[] = "RotaryEncoder"; // Name of module for manual override UI and state machine assembler

// Output stream setup
char moduleStreamPrefix = 'M'; // Command character to send before each position value when streaming data via output stream jack
byte outputStreamDatatype = 'H'; // Integer type to use when streaming position value. 'H' = 16-bit unsigned int, 'L' = 32-bit unsigned int

// Hardware setup
const byte EncoderPinA = 35;
const byte EncoderPinB = 36;
const byte EncoderPinZ = 37;

// Parameters
const byte maxThresholds = 8;
int16_t thresholds[maxThresholds] = {0}; // Initialized by client. Position range = -512 : 512 encoder tics, corresponding to -180 : +180 degrees
boolean thresholdActive[maxThresholds] = {true}; // Thresholds are inactivated on crossing, until manually reset
byte nThresholds = maxThresholds; // Number of thresholds currently used
int16_t wrapPoint = 512; // Position used to wrap position. At 512, if position > 512 or < -512, position wraps to -512 and 512 respectively.
int nWraps = 0; // number of times (positive or negative) that the wheel position has wrapped since last reset

// State variables
boolean usbStreaming = false; // If currently streaming position and time data to the output-stream port
boolean sendEvents = true; // True if sending threshold crossing events to state machine
boolean isLogging = false; // If currently logging position and time to microSD memory
boolean moduleStreaming = false; // If streaming position to a separate module via the output stream jack (preconfigured output for DDS module)
int16_t EncoderPos = 0; // Current position of the rotary encoder
byte currentDir = 0; // Current direction (0 = clockwise, 1 = counterclockwise)

// Program variables
byte opCode = 0;
byte opSource = 0;
byte param = 0;
boolean newOp = false;
boolean loggedDataAvailable = 0;
boolean wrappingEnabled = true;
byte wrapMode = 0;
boolean EncoderPinAValue = 0;
boolean LastEncoderPinAValue = 0;
boolean EncoderPinBValue = 0;
uint32_t dataPos = 0;
uint32_t dataMax = 4294967295; // Maximim number of positions that can be logged (limited by 32-bit counter)
elapsedMicros currentTime;
int16_t wrapPointInverse = 0;
union {
    byte uint8[4];
    uint16_t uint16;
    uint32_t uint32;
} typeBuffer;

// microSD variables
uint32_t nRemainderBytes = 0;
uint32_t nFullBufferReads = 0;
union {
    byte uint8[800];
    uint32_t int32[200];
} sdWriteBuffer;
const uint32_t sdReadBufferSize = 2048; // in bytes
uint8_t sdReadBuffer[sdReadBufferSize] = {0};

// Interrupt and position buffers
const uint32_t positionBufferSize = 1024;
int16_t positionBuffer[positionBufferSize][2] = {0};
int32_t timeBuffer[positionBufferSize][2] = {0};
uint32_t iPositionBuffer[2] = {0};
uint32_t thisInd = 0;
boolean positionBufferFlag = false;
byte currentPositionBuffer = 0;
byte thisPositionBuffer = 0;
uint32_t currentInd = 0;

// Event buffer
byte newEventType = 0; // 0 = State Machine, (Not yet implemented: 1 = TTL Ch 18, 2 = TTL Ch 19, 3 = I2C)
byte newEventCode = 0; // The new event (if state machine or I2C, a byte; if TTL, a logic level)
uint32_t newEventTime = 0;
boolean newEvent = 0;

// USB streaming buffer
const uint32_t usbStreamingBufferSize = 1000;
uint32_t usbStreamingBufferPos = 0;
byte usbStreamingBuffer[usbStreamingBufferSize] = {0};
uint32_t nPositions = 0;

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(115200);
  Serial3.begin(1312500);
  Serial2.begin(1312500);
  pinMode(EncoderPinA, INPUT);
  pinMode (EncoderPinB, INPUT);
  SPI.begin();
  SD.begin(); // Initialize microSD card
  SD.remove("Data.wfm");
  DataFile = SD.open("Data.wfm", FILE_WRITE);
  wrapPointInverse = wrapPoint * -1;
  attachInterrupt(EncoderPinA, updatePosition, CHANGE);
}

void loop() {
  if (myUSB.available() > 0) {
    opCode = myUSB.readByte();
    newOp = true;
    opSource = 0;
  } else if (StateMachineCOM.available() > 0) {
    opCode = StateMachineCOM.readByte();
    newOp = true;
    opSource= 1;
  } else if (OutputStreamCOM.available() > 0) {
    opCode = OutputStreamCOM.readByte();
    newOp = true;
    opSource = 2;
  }
  if (newOp) {
    newOp = false;
    switch(opCode) {
      case 255: // Return module name and info
        if (opSource == 1) { // If requested by state machine
          returnModuleInfo();
        }
      break;
      case 'C': // USB Handshake
        if (opSource == 0) {
          myUSB.writeByte(217);
        }
      break;
      case 'S': // Start/Stop USB position+time data stream
        if (opSource == 0) {
          usbStreaming = myUSB.readByte();
          if (usbStreaming) {
            EncoderPos = 0; // Reset position
            nWraps = 0; // Reset wrap counter
            currentTime = 0; // Reset clock
          }
        }
      break;
      case 'O': // Start/Stop module position data stream
        moduleStreaming = readByteFromSource(opSource);
        if (opSource == 0) {
          myUSB.writeByte(1); // Confirm
        }
      break;
      case 'V': // Set event transmission to state machine (on/off)
      if (opSource == 0) {
        sendEvents = myUSB.readByte();
        myUSB.writeByte(1);
      }
      break;
      case '#': // Log an incoming event and the current time
        if (opSource == 1) {
          newEventCode = StateMachineCOM.readByte();
          newEventTime = currentTime;
          newEventType = 0; // 0 = State Machine, (Not yet implemented: 1 = TTL Ch 18, 2 = TTL Ch 19, 3 = I2C)
          if (usbStreaming) {
            usbStreamingBuffer[0] = 'E';
            usbStreamingBuffer[1] = newEventType; // State machine event type (0 = 
            usbStreamingBuffer[2] = newEventCode; // State machine event code
            typeBuffer.uint32 = newEventTime; // State machine event time on RE module clock
            usbStreamingBuffer[3] = typeBuffer.uint8[0];
            usbStreamingBuffer[4] = typeBuffer.uint8[1];
            usbStreamingBuffer[5] = typeBuffer.uint8[2];
            usbStreamingBuffer[6] = typeBuffer.uint8[3];
            myUSB.writeByteArray(usbStreamingBuffer, 7);
          }
          newEvent = true;
        }
      break;
      case 'W': // Set wrap point (in tics)
      if (opSource == 0) {
        wrapPoint = myUSB.readInt16();
        myUSB.writeByte(1);
        if (wrapPoint != 0) {
          wrappingEnabled = true;
        } else {
          wrappingEnabled = false;
        }
        wrapPointInverse = wrapPoint * -1;
        nWraps = 0;
        if ((EncoderPos > wrapPoint) || (EncoderPos < wrapPointInverse)) {
          EncoderPos = wrapPoint;
        }
      }
      break;
      case 'M': // Set wrap Mode: 0 bipolar (wrap to negative wrapPoint), 1 = unipolar (wrap to zero)
        if (opSource == 0) {
          wrapMode = myUSB.readByte(); // Read number of thresholds to program
          myUSB.writeByte(1);
        }
      break;
      case 'T': // Program thresholds
        if (opSource == 0) {
          param = myUSB.readByte(); // Read number of thresholds to program
          if (param <= maxThresholds) {
            nThresholds = param;
            for (int i = 0; i < nThresholds; i++) {
              thresholds[i] = myUSB.readInt16();
            }
            myUSB.writeByte(1);
          } else {
            myUSB.writeByte(0);
          }
        }
      break;
      case 'I': // Set 1-character prefix preceding each position data point streamed to a receiving Bpod module
        if (opSource == 0) {
          moduleStreamPrefix = myUSB.readByte();
          myUSB.writeByte(1);
        }
      break;
      case ';': // Set enable/disable status of all thresholds
        param = readByteFromSource(opSource);
        for (int i = 0; i < nThresholds; i++) {
          thresholdActive[i] = bitRead(param, i);
        }
        nWraps = 0;
      break;
      case 'Z': // Zero position
          EncoderPos = 0;
          nWraps = 0;
          if (usbStreaming) {
            newEventTime = currentTime;
            usbStreamingBuffer[0] = 'P'; // Code for position data
            usbStreamingBuffer[1] = 0;
            usbStreamingBuffer[2] = 0;
            typeBuffer.uint32 = newEventTime; // Time
            usbStreamingBuffer[3] = typeBuffer.uint8[0]; 
            usbStreamingBuffer[4] = typeBuffer.uint8[1];
            usbStreamingBuffer[5] = typeBuffer.uint8[2];
            usbStreamingBuffer[6] = typeBuffer.uint8[3];
            myUSB.writeByteArray(usbStreamingBuffer, 7);
          }
      break;
      case 'E': // Enable all thresholds
        for (int i = 0; i < nThresholds; i++) {
          thresholdActive[i] = true;
        }
        nWraps = 0;
      break;
      case 'L': // Start microSD logging
        startLogging();
      break;
      case 'F': // finish microSD logging
        logCurrentPosition();
        isLogging = false;
        if (dataPos > 0) {
          loggedDataAvailable = true;
        }
      break;
      case 'R': // Return logged data
        if (opSource == 0) {
          isLogging = false;
          if (loggedDataAvailable) {
            loggedDataAvailable = false;
            DataFile.seek(0);
            if (dataPos*8 > sdReadBufferSize) {
              nFullBufferReads = (unsigned long)(floor(((double)dataPos)*8 / (double)sdReadBufferSize));
            } else {
              nFullBufferReads = 0;
            }
            myUSB.writeUint32(dataPos);     
            for (int i = 0; i < nFullBufferReads; i++) { // Full buffer transfers; skipped if nFullBufferReads = 0
              DataFile.read(sdReadBuffer, sdReadBufferSize);
              
              myUSB.writeByteArray(sdReadBuffer, sdReadBufferSize);
            }
            nRemainderBytes = (dataPos*8)-(nFullBufferReads*sdReadBufferSize);
            if (nRemainderBytes > 0) {
              DataFile.read(sdReadBuffer, nRemainderBytes);
              myUSB.writeByteArray(sdReadBuffer, nRemainderBytes);     
            }              
            dataPos = 0;
          } else {
            myUSB.writeUint32(0);
          }
        }
      break;
      case 'Q': // Return current encoder position
        if (opSource == 0) {
          myUSB.writeInt16(EncoderPos);
        }
      break;
      case 'P': // Set current encoder position
        if (opSource == 0) {
          EncoderPos = myUSB.readInt16();
          nWraps = 0;
          myUSB.writeByte(1);
        }
      break;
      case 'X': // Reset all data streams
        resetDataStreams();
      break;
    } // End switch(opCode)
  } // End if (SerialUSB.available())

  if(positionBufferFlag) { // If new data points have been added since last loop
    positionBufferFlag = false;
    nPositions = iPositionBuffer[currentPositionBuffer];
    thisPositionBuffer = currentPositionBuffer;
    currentPositionBuffer = 1-currentPositionBuffer;
    iPositionBuffer[currentPositionBuffer] = 0;
    if (usbStreaming) {
      if (nPositions > 0) {
        usbStreamingBufferPos = 0;
        for (int i = 0; i < nPositions; i++) {
          usbStreamingBuffer[usbStreamingBufferPos] = 'P'; // Code for position data
          usbStreamingBufferPos++;
          typeBuffer.uint16 = positionBuffer[i][thisPositionBuffer]; // Position
          usbStreamingBuffer[usbStreamingBufferPos] = typeBuffer.uint8[0];
          usbStreamingBuffer[usbStreamingBufferPos+1] = typeBuffer.uint8[1];
          usbStreamingBufferPos += 2;
          typeBuffer.uint32 = timeBuffer[i][thisPositionBuffer]; // Time
          usbStreamingBuffer[usbStreamingBufferPos] = typeBuffer.uint8[0]; 
          usbStreamingBuffer[usbStreamingBufferPos+1] = typeBuffer.uint8[1];
          usbStreamingBuffer[usbStreamingBufferPos+2] = typeBuffer.uint8[2];
          usbStreamingBuffer[usbStreamingBufferPos+3] = typeBuffer.uint8[3];
          usbStreamingBufferPos+=4;
        }
        myUSB.writeByteArray(usbStreamingBuffer, usbStreamingBufferPos);
      }
      myUSB.flush();
    }
    if (isLogging) {
      if (dataPos<dataMax) {
        logCurrentPosition();
      }
    } 
    if (moduleStreaming) {
      for (int i = 0; i < nPositions; i++) {
        OutputStreamCOM.writeByte(moduleStreamPrefix);
        if (wrapMode == 0) { // In Bipolar mode, sends the unipolar equivalent
          typeBuffer.uint32 = positionBuffer[i][thisPositionBuffer]+wrapPoint;
        } else {
          typeBuffer.uint32 = positionBuffer[i][thisPositionBuffer];
        }
        switch(outputStreamDatatype) {
          case 'H':
            OutputStreamCOM.writeUint16(typeBuffer.uint16);
          break;
          case 'L':
            OutputStreamCOM.writeUint32(typeBuffer.uint32);
          break;
        }
      }
    }
    if (sendEvents) {
      if (nWraps == 0) { // Thresholds are only defined within +/- the range of the wrap point
        for (int i = 0; i < nThresholds; i++) {
          if (thresholdActive[i]) {
             if (thresholds[i] < 0) {
                for (int j = 0; j < nPositions; j++) {
                  if (thresholdActive[i]) {
                    if (positionBuffer[j][thisPositionBuffer] <= thresholds[i]) {
                      thresholdActive[i] = false;
                      StateMachineCOM.writeByte(i+1);
                    }
                  }
                }
             } else {
                for (int j = 0; j < nPositions; j++) {
                  if (thresholdActive[i]) {
                    if (positionBuffer[j][thisPositionBuffer] >= thresholds[i]) {
                      thresholdActive[i] = false;
                      StateMachineCOM.writeByte(i+1);
                    }
                  }
                }
             }
          }
        }
      }
    }
  }
}

void updatePosition() { // Implements 'X1 encoding' as per NI encoder tutorial: http://www.ni.com/tutorial/7109/en/
  // This interrupt handler is called each time the value of pin A changes
  EncoderPinAValue = digitalReadFast(EncoderPinA);
  EncoderPinBValue = digitalReadFast(EncoderPinB);
  if (EncoderPinAValue && !LastEncoderPinAValue) { // If rising edge of pin A
    if (EncoderPinBValue == HIGH) {
      if (currentDir == 0) {
        EncoderPos++;
        processPosition();
      }
      currentDir = 0;
    } else {
      currentDir = 1;
    }
  } else {                                         // If falling edge of pin A
    if (EncoderPinBValue == HIGH) {
      if (currentDir == 1) {
        EncoderPos--;
        processPosition();
      }
      currentDir = 1;
    } else {
      currentDir = 0;
    }
  }
  LastEncoderPinAValue = EncoderPinAValue;
}

void processPosition() {
  if (wrappingEnabled) {
    switch (wrapMode) {
      case 0: // Bipolar mode
        if (EncoderPos <= wrapPointInverse) {
          EncoderPos = wrapPoint; nWraps--;
        } else if (EncoderPos >= wrapPoint) {
          EncoderPos = wrapPointInverse; nWraps++;
        }
      break;
      case 1: // Unipolar mode
        if (EncoderPos > wrapPoint) {
          EncoderPos = 0; nWraps++;
        } else if (EncoderPos < 0) { 
          EncoderPos = wrapPoint; nWraps--;
        }
      break;
    }
  }
  thisInd = iPositionBuffer[currentPositionBuffer];
  positionBuffer[thisInd][currentPositionBuffer] = EncoderPos;
  timeBuffer[thisInd][currentPositionBuffer] = currentTime;
  iPositionBuffer[currentPositionBuffer]++;
  positionBufferFlag = true;
}

void returnModuleInfo() {
  StateMachineCOM.writeByte(65); // Acknowledge
  StateMachineCOM.writeUint32(FirmwareVersion); // 4-byte firmware version
  StateMachineCOM.writeByte(sizeof(moduleName)-1); // Length of module name
  StateMachineCOM.writeCharArray(moduleName, sizeof(moduleName)-1); // Module name
  StateMachineCOM.writeByte(0); // 1 if more info follows, 0 if not
}

void startLogging() {
  DataFile.seek(0);
  dataPos = 0;
  currentTime = 0;
  isLogging = true;
  iPositionBuffer[0] = 0;
  iPositionBuffer[1] = 0;
}

void logCurrentPosition() {
  if (nPositions > 0) {
    for (int i = 0; i < nPositions; i++) {
      sdWriteBuffer.int32[0] = positionBuffer[i][thisPositionBuffer];
      sdWriteBuffer.int32[1] = timeBuffer[i][thisPositionBuffer];
      DataFile.write(sdWriteBuffer.uint8, 8);
      dataPos+=1;
    }
  }
}

byte readByteFromSource(byte opSource) {
  switch (opSource) {
    case 0:
      return myUSB.readByte();
    break;
    case 1:
      return StateMachineCOM.readByte();
    break;
    case 2:
      return OutputStreamCOM.readByte();
    break;
  }
}

void resetDataStreams() {
  usbStreaming = false;
  isLogging = false;
  dataPos = 0;
  EncoderPos = 0;
  nWraps = 0;
  iPositionBuffer[0] = 0;
  iPositionBuffer[1] = 0;
}
