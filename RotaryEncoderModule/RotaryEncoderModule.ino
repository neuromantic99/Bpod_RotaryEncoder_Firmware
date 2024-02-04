/*
  ----------------------------------------------------------------------------

  This file is part of the Sanworks Bpod_Gen2 repository
  Copyright (C) 2022 Sanworks LLC, Rochester, New York, USA

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

// The rotary encoder module, powered by Teensy 3.5 (v1) or 4.0 (v2), interfaces the Bpod state machine with a quadrature rotary encoder (e.g. Yumo E6B2-CWZ3E)
// The serial interface allows the state machine to set position thresholds which generate Bpod events when crossed, and to reset the encoder position.
// The MATLAB interface can capture streaming position data and set new thresholds.

#define FirmwareVersion 6
#define HARDWARE_VERSION 1 // NOTE: SET THIS TO MATCH THE TARGET VERSION OF THE MODULE! You MUST uncomment the next line if compiling for HARDWARE_VERSION 2
// #include "QuadEncoder.h"

#include "ArCOM.h"
#include <SPI.h>
#include "SdFat.h"

#if HARDWARE_VERSION == 1
SdFs SDcard;
FsFile DataFile;    // File on microSD card, to store waveform data
bool ready = false; // Indicates if SD is busy (for use with SDBusy() funciton)
#else
QuadEncoder hwEnc(1, 7, 8);
IntervalTimer hardwareTimer;
#endif
ArCOM myUSB(SerialUSB); // USB is an ArCOM object. ArCOM wraps Arduino's SerialUSB interface

#if HARDWARE_VERSION == 1
ArCOM StateMachineCOM(Serial3); // UART serial port
ArCOM OutputStreamCOM(Serial2); // UART serial port
#elif HARDWARE_VERSION == 2
ArCOM StateMachineCOM(Serial1);
#endif
// Module setup
char moduleName[] = "RotaryEncoder"; // Name of module for manual override UI and state machine assembler

// Output stream setup
char moduleStreamPrefix = 'M';   // Command character to send before each position value when streaming data via output stream jack
byte outputStreamDatatype = 'H'; // Integer type to use when streaming position value. 'H' = 16-bit unsigned int, 'L' = 32-bit unsigned int

// Hardware setup
#if HARDWARE_VERSION == 1
const byte EncoderPinA = 35;
const byte EncoderPinB = 36;
const byte EncoderPinZ = 37;
byte circuitRevisionArray[5] = {26, 27, 28, 29, 30};
#elif HARDWARE_VERSION == 2
const byte EncoderPinA = 8;
const byte EncoderPinB = 7;
const byte EncoderPinZ = 6;
byte circuitRevisionArray[5] = {17, 18, 19, 20, 21};
#endif

#if HARDWARE_VERSION == 2
const byte DAC_CS_Pin = 10;
SPISettings DACSettings(30000000, MSBFIRST, SPI_MODE0); // Settings for DAC
byte dacBuffer[3] = {0};                                // Holds bytes to be written to the DAC
union
{
  byte byteArray[4];
  uint16_t uint16[2];
} dacValue;
#endif

// Constants
const uint16_t logic3v3_bits = 52748; // Bits for 3.3V on 16-bit DAC register spanning 0-4.1V

// Parameters
const byte maxThresholds = 8;
int16_t thresholds[maxThresholds] = {0};               // Initialized by client. Position range = -512 : 512 encoder tics, corresponding to -180 : +180 degrees
boolean thresholdActive[maxThresholds] = {true};       // Thresholds are inactivated on crossing, until manually reset
byte thresholdTypes[maxThresholds] = {0};              // Type 0 (Threshold reached when position crossed) Type 1 (After enable, threshold reached after Time spent within +/- position)
uint32_t thresholdTimes[maxThresholds] = {0};          // For thresholdType 1, time parameter. Units = # of 100μs hardware timer cycles.
int16_t nextTrialThresholds[maxThresholds] = {0};      // Thresholds for next trial (thresholds is set to this on next call to the '*' (push) command)
byte nextTrialThresholdTypes[maxThresholds] = {0};     // Threshold types for next trial (thresholdTypes is set to this on next call to the '*' (push) command)
uint32_t nextTrialThresholdTimes[maxThresholds] = {0}; // Threshold times for next trial (thresholdTimes is set to this on next call to the '*' (push) command)
byte nThresholds = maxThresholds;                      // Number of thresholds currently used
byte nextTrialnThresholds = 0;                         // Number of thresholds for next trial (nThresholds is set to this on next call to the '*' (push) command)
#if HARDWARE_VERSION == 1
int16_t wrapPoint = 512; // Position used to wrap position. At 512, if position > 512 or < -512, position wraps to -512 and 512 respectively.
#else
int16_t wrapPoint = 2048;
#endif
int nWraps = 0; // number of times (positive or negative) that the wheel position has wrapped since last reset

// State variables
boolean usbStreaming = false;        // If currently streaming position and time data to the output-stream port
boolean sendEvents = true;           // True if sending threshold crossing events to state machine
boolean isLogging = false;           // If currently logging position and time to microSD memory
boolean moduleStreaming = false;     // If streaming position to a separate module via the output stream jack (preconfigured output for DDS module)
boolean newThresholdsLoaded = false; // If new 'advanced' thresholds were loaded to the device with 't' command (different from legacy 'T')
int16_t EncoderPos = 0;              // Current position of the rotary encoder
int16_t LastEncoderPos = 0;          // Last known position of rotary encoder
byte currentDir = 0;                 // Current direction (0 = clockwise, 1 = counterclockwise)
byte outputSyncLogic[2] = {0};

// Program variables
byte testByte = 0;
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
uint32_t thresholdTimeElapsed[maxThresholds] = {0}; // Current time (number of timer cycles) since threshold was enabled
int16_t ThresholdPos[maxThresholds] = {0};          // A separate position value for time thresholds (time spent within position range; threshold type 1)
int16_t EncoderPosChange = 0;                       // Change in position, used to reset timer when using time thresholds
union
{
  byte uint8[4];
  uint16_t uint16;
  uint32_t uint32;
} typeBuffer;

#if HARDWARE_VERSION == 1
// microSD variables
uint32_t nRemainderBytes = 0;
uint32_t nFullBufferReads = 0;
union
{
  byte uint8[800];
  uint32_t int32[200];
} sdWriteBuffer;
const uint32_t sdReadBufferSize = 2048; // in bytes
uint8_t sdReadBuffer[sdReadBufferSize] = {0};
#endif

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
byte circuitRevision = 0; // Minor revision of circuit board
uint16_t TestVal = 0;

void setup()
{
  // put your setup code here, to run once:
  SerialUSB.begin(115200);
#if HARDWARE_VERSION == 1
  Serial3.begin(1312500);
  Serial2.begin(1312500);
#elif HARDWARE_VERSION == 2
  Serial1.begin(1312500);
#endif
  SPI.begin();
#if HARDWARE_VERSION == 1
  pinMode(EncoderPinA, INPUT);
  pinMode(EncoderPinB, INPUT);
  SDcard.begin(SdioConfig(FIFO_SDIO));
  SDcard.remove("Data.wfm");
  DataFile = SDcard.open("Data.wfm", O_RDWR | O_CREAT);
  DataFile.preAllocate(1000000);
  while (sdBusy())
  {
  }
  DataFile.seek(0);
#endif
#if HARDWARE_VERSION == 2
  hwEnc.setInitConfig();
  hwEnc.init();
  pinMode(DAC_CS_Pin, OUTPUT);
  digitalWrite(DAC_CS_Pin, HIGH);
  SPI.beginTransaction(DACSettings);
  powerUpDAC();
#endif
  wrapPointInverse = wrapPoint * -1;
  // Read hardware revision from circuit board (an array of grounded pins indicates revision in binary, grounded = 1, floating = 0)
  circuitRevision = 0;
  for (int i = 0; i < 5; i++)
  {
    pinMode(circuitRevisionArray[i], INPUT_PULLUP);
    circuitRevision += pow(2, i) * digitalRead(circuitRevisionArray[i]);
    pinMode(circuitRevisionArray[i], INPUT);
  }
  circuitRevision = 31 - circuitRevision;
#if HARDWARE_VERSION == 1
  attachInterrupt(EncoderPinA, updatePosition, CHANGE);
#else
  hardwareTimer.begin(updatePosition, 100);
#endif
}

void loop()
{
  if (myUSB.available() > 0)
  {
    opCode = myUSB.readByte();
    newOp = true;
    opSource = 0;
  }
  else if (StateMachineCOM.available() > 0)
  {
    opCode = StateMachineCOM.readByte();
    newOp = true;
    opSource = 1;
#if HARDWARE_VERSION == 1
  }
  else if (OutputStreamCOM.available() > 0)
  {
    opCode = OutputStreamCOM.readByte();
    newOp = true;
    opSource = 2;
#endif
  }
  if (newOp)
  {
    newOp = false;
    switch (opCode)
    {
    case 255: // Return module name and info
      if (opSource == 1)
      { // If requested by state machine
        returnModuleInfo();
      }
      break;
#if HARDWARE_VERSION == 1
    case 254: // Relay test byte from USB to echo module, or from echo module back to USB
      if (opSource == 0)
      {
        OutputStreamCOM.writeByte(254);
      }
      if (opSource == 2)
      {
        myUSB.writeByte(254);
      }
      break;
#endif
    case 'C': // USB Handshake
      if (opSource == 0)
      {
        myUSB.writeByte(217);
      }
      break;
#if HARDWARE_VERSION == 1
    case 200: // Byte forwarding (Diagnostic)
      switch (opSource)
      {
      case 0:
        testByte = myUSB.readByte();
        OutputStreamCOM.writeByte(testByte);
        break;
      case 1:
        testByte = myUSB.readByte();
        StateMachineCOM.writeByte(testByte);
        break;
      }
      break;
    case 201: // Byte forwarding (Diagnostic)
      switch (opSource)
      {
      case 2:
        myUSB.writeByte(201);
        break;
      }
      break;
#endif
    case 'S': // Start/Stop USB position+time data stream
      if (opSource == 0)
      {
        usbStreaming = myUSB.readByte();
        if (usbStreaming)
        {
          EncoderPos = 0;  // Reset position
          nWraps = 0;      // Reset wrap counter
          currentTime = 0; // Reset clock
        }
      }
      break;
    case 'O': // Start/Stop module position data stream
      moduleStreaming = readByteFromSource(opSource);
      if (opSource == 0)
      {
        myUSB.writeByte(1); // Confirm
      }
      break;
    case 'V': // Set event transmission to state machine (on/off)
      if (opSource == 0)
      {
        sendEvents = myUSB.readByte();
        myUSB.writeByte(1);
      }
      break;
    case '#': // Log an incoming event and the current time
      if (opSource == 1)
      {
        newEventCode = StateMachineCOM.readByte();
        newEventTime = currentTime;
        newEventType = 0; // 0 = State Machine, (Not yet implemented: 1 = TTL Ch 18, 2 = TTL Ch 19, 3 = I2C)
        if (usbStreaming)
        {
          usbStreamingBuffer[0] = 'E';
          usbStreamingBuffer[1] = newEventType; // State machine event type (0 =
          usbStreamingBuffer[2] = newEventCode; // State machine event code
          typeBuffer.uint32 = newEventTime;     // State machine event time on RE module clock
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
      if (opSource == 0)
      {
        wrapPoint = myUSB.readInt16();
        myUSB.writeByte(1);
        if (wrapPoint != 0)
        {
          wrappingEnabled = true;
        }
        else
        {
          wrappingEnabled = false;
        }
        wrapPointInverse = wrapPoint * -1;
        nWraps = 0;
        if ((EncoderPos > wrapPoint) || (EncoderPos < wrapPointInverse))
        {
          EncoderPos = wrapPoint;
        }
      }
      break;
    case 'M': // Set wrap Mode: 0 bipolar (wrap to negative wrapPoint), 1 = unipolar (wrap to zero)
      if (opSource == 0)
      {
        wrapMode = myUSB.readByte(); // Read number of thresholds to program
        myUSB.writeByte(1);
      }
      break;
    case 'T': // Program simple position thresholds (effective immediately)
      if (opSource == 0)
      {
        param = myUSB.readByte(); // Read number of thresholds to program
        if (param <= maxThresholds)
        {
          nThresholds = param;
          for (int i = 0; i < nThresholds; i++)
          {
            thresholds[i] = myUSB.readInt16();
            thresholdTypes[i] = 0;
            thresholdTimes[i] = 0;
          }
          myUSB.writeByte(1);
        }
        else
        {
          myUSB.writeByte(0);
        }
      }
      break;
    case 't': // Program 'advanced' thresholds to hold in storage buffer until next call to '*' (the 'push' command). HW version 2 only!
              // No confirmation byte is returned, for compatability with whole-session USB position streaming
              // Advanced thresholds can be: Type 0 (Threshold reached when position crossed) Type 1 (After enable, threshold reached after Time spent within +/- position)
#if HARDWARE_VERSION > 1
      if (opSource == 0)
      {
        param = myUSB.readByte();
        if (param <= maxThresholds)
        { // Read number of thresholds to program
          nextTrialnThresholds = param;
          myUSB.readByteArray(nextTrialThresholdTypes, nextTrialnThresholds);
          myUSB.readInt16Array(nextTrialThresholds, nextTrialnThresholds);
          myUSB.readUint32Array(nextTrialThresholdTimes, nextTrialnThresholds); // Units = HW timer cycles (# of 100μs intervals)
          newThresholdsLoaded = true;
        }
      }
#endif
      break;
    case '*': // The 'Push' command sets any advanced thresholds previously loaded with the 't' command to be the current thresholds
      if (newThresholdsLoaded)
      {
        for (int i = 0; i < nextTrialnThresholds; i++)
        {
          thresholds[i] = nextTrialThresholds[i];
          thresholdTypes[i] = nextTrialThresholdTypes[i];
          thresholdTimes[i] = nextTrialThresholdTimes[i];
        }
        nThresholds = nextTrialnThresholds;
        newThresholdsLoaded = false;
      }
      break;
    case 'I': // Set 1-character prefix preceding each position data point streamed to a receiving Bpod module
      if (opSource == 0)
      {
#if HARDWARE_VERSION == 1
        moduleStreamPrefix = myUSB.readByte();
        myUSB.writeByte(1);
#else
        myUSB.writeByte(0);
#endif
      }
      break;
    case ';': // Set enable/disable status of all thresholds
      param = readByteFromSource(opSource);
      for (int i = 0; i < nThresholds; i++)
      {
        thresholdActive[i] = bitRead(param, i);
        if (thresholdActive[i])
        {
          thresholdTimeElapsed[i] = 0;
          ThresholdPos[i] = 0;
        }
      }
      nWraps = 0;
      break;
    case 'Z': // Zero position
      EncoderPos = 0;
#if HARDWARE_VERSION == 2
      hwEnc.write(0);
#endif
      nWraps = 0;
      if (usbStreaming)
      {
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
      for (int i = 0; i < nThresholds; i++)
      {
        thresholdActive[i] = true;
        ThresholdPos[i] = 0;
        thresholdTimeElapsed[i] = 0;
      }
      nWraps = 0;
      break;
    case 'L': // Start microSD logging
#if HARDWARE_VERSION == 1
      startLogging();
#endif
      break;
    case 'F': // finish microSD logging
#if HARDWARE_VERSION == 1
      logCurrentPosition();
      isLogging = false;
      if (dataPos > 0)
      {
        loggedDataAvailable = true;
      }
#endif
      break;
    case 'R': // Return logged data
      if (opSource == 0)
      {
#if HARDWARE_VERSION == 1
        isLogging = false;
        if (loggedDataAvailable)
        {
          loggedDataAvailable = false;
          DataFile.seek(0);
          if (dataPos * 8 > sdReadBufferSize)
          {
            nFullBufferReads = (unsigned long)(floor(((double)dataPos) * 8 / (double)sdReadBufferSize));
          }
          else
          {
            nFullBufferReads = 0;
          }
          myUSB.writeUint32(dataPos);
          for (int i = 0; i < nFullBufferReads; i++)
          { // Full buffer transfers; skipped if nFullBufferReads = 0
            DataFile.read(sdReadBuffer, sdReadBufferSize);

            myUSB.writeByteArray(sdReadBuffer, sdReadBufferSize);
          }
          nRemainderBytes = (dataPos * 8) - (nFullBufferReads * sdReadBufferSize);
          if (nRemainderBytes > 0)
          {
            DataFile.read(sdReadBuffer, nRemainderBytes);
            myUSB.writeByteArray(sdReadBuffer, nRemainderBytes);
          }
          dataPos = 0;
        }
        else
        {
          myUSB.writeUint32(0);
        }
#endif
      }
      break;
    case 'Q': // Return current encoder position
      if (opSource == 0)
      {
        myUSB.writeInt16(EncoderPos);
      }
      break;
    case 'P': // Set current encoder position
      if (opSource == 0)
      {
        EncoderPos = myUSB.readInt16();
#if HARDWARE_VERSION == 2
        hwEnc.write((uint32_t)EncoderPos);
#endif
        nWraps = 0;
        myUSB.writeByte(1);
      }
      break;
    case 'X': // Reset all data streams
      resetDataStreams();
      break;
    } // End switch(opCode)
  }   // End if (SerialUSB.available())

  if (positionBufferFlag)
  { // If new data points have been added since last loop
    positionBufferFlag = false;
    nPositions = iPositionBuffer[currentPositionBuffer];
    thisPositionBuffer = currentPositionBuffer;
    currentPositionBuffer = 1 - currentPositionBuffer;
    iPositionBuffer[currentPositionBuffer] = 0;
    if (usbStreaming)
    {
      if (nPositions > 0)
      {
        usbStreamingBufferPos = 0;
        for (int i = 0; i < nPositions; i++)
        {
          usbStreamingBuffer[usbStreamingBufferPos] = 'P'; // Code for position data
          usbStreamingBufferPos++;
          typeBuffer.uint16 = positionBuffer[i][thisPositionBuffer]; // Position
          usbStreamingBuffer[usbStreamingBufferPos] = typeBuffer.uint8[0];
          usbStreamingBuffer[usbStreamingBufferPos + 1] = typeBuffer.uint8[1];
          usbStreamingBufferPos += 2;
          typeBuffer.uint32 = timeBuffer[i][thisPositionBuffer]; // Time
          usbStreamingBuffer[usbStreamingBufferPos] = typeBuffer.uint8[0];
          usbStreamingBuffer[usbStreamingBufferPos + 1] = typeBuffer.uint8[1];
          usbStreamingBuffer[usbStreamingBufferPos + 2] = typeBuffer.uint8[2];
          usbStreamingBuffer[usbStreamingBufferPos + 3] = typeBuffer.uint8[3];
          usbStreamingBufferPos += 4;
        }
        myUSB.writeByteArray(usbStreamingBuffer, usbStreamingBufferPos);
      }
      myUSB.flush();
    }
#if HARDWARE_VERSION == 1
    if (isLogging)
    {
      if (dataPos < dataMax)
      {
        logCurrentPosition();
      }
    }
    if (moduleStreaming)
    {
      for (int i = 0; i < nPositions; i++)
      {
        OutputStreamCOM.writeByte(moduleStreamPrefix);
        if (wrapMode == 0)
        { // In Bipolar mode, sends the unipolar equivalent
          typeBuffer.uint32 = positionBuffer[i][thisPositionBuffer] + wrapPoint;
        }
        else
        {
          typeBuffer.uint32 = positionBuffer[i][thisPositionBuffer];
        }
        switch (outputStreamDatatype)
        {
        case 'H':
          OutputStreamCOM.writeUint16(typeBuffer.uint16);
          break;
        case 'L':
          OutputStreamCOM.writeUint32(typeBuffer.uint32);
          break;
        }
      }
    }
#endif
    if (sendEvents)
    {
      if (nWraps == 0)
      { // Thresholds are only defined within +/- the range of the wrap point
        for (int i = 0; i < nThresholds; i++)
        {
          if (thresholdActive[i])
          {
            if (thresholdTypes[i] == 0)
            {
              if (thresholds[i] < 0)
              {
                for (int j = 0; j < nPositions; j++)
                {
                  if (thresholdActive[i])
                  {
                    if (positionBuffer[j][thisPositionBuffer] <= thresholds[i])
                    {
                      thresholdActive[i] = false;
                      StateMachineCOM.writeByte(i + 1);
                    }
                  }
                }
              }
              else
              {
                for (int j = 0; j < nPositions; j++)
                {
                  if (thresholdActive[i])
                  {
                    if (positionBuffer[j][thisPositionBuffer] >= thresholds[i])
                    {
                      thresholdActive[i] = false;
                      StateMachineCOM.writeByte(i + 1);
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
}

void updatePosition()
{
// This interrupt handler is called each time the value of pin A changes (HW version 1) or on each timer callback (HW version 2)
#if HARDWARE_VERSION == 1
  // Implements 'X1 encoding' as per NI encoder tutorial: http://www.ni.com/tutorial/7109/en/
  EncoderPinAValue = digitalReadFast(EncoderPinA);
  EncoderPinBValue = digitalReadFast(EncoderPinB);
  if (EncoderPinAValue && !LastEncoderPinAValue)
  { // If rising edge of pin A
    if (EncoderPinBValue == HIGH)
    {
      if (currentDir == 0)
      {
        EncoderPos++;
        processPosition();
      }
      currentDir = 0;
    }
    else
    {
      currentDir = 1;
    }
  }
  else
  { // If falling edge of pin A
    if (EncoderPinBValue == HIGH)
    {
      if (currentDir == 1)
      {
        EncoderPos--;
        processPosition();
      }
      currentDir = 1;
    }
    else
    {
      currentDir = 0;
    }
  }
  LastEncoderPinAValue = EncoderPinAValue;
#elif HARDWARE_VERSION == 2
  EncoderPos = hwEnc.read();
  EncoderPosChange = 0;
  if (EncoderPos != LastEncoderPos)
  {
    processPosition();
    EncoderPosChange = EncoderPos - LastEncoderPos;
    LastEncoderPos = EncoderPos;
  }
  for (int i = 0; i < nThresholds; i++)
  {
    if (thresholdTypes[i] == 1)
    {
      if (thresholdActive[i])
      {
        thresholdTimeElapsed[i]++;
        ThresholdPos[i] += EncoderPosChange;
        if (abs(ThresholdPos[i]) >= abs(thresholds[i]))
        { // Reset threshold position and time
          ThresholdPos[i] = 0;
          thresholdTimeElapsed[i] = 0;
        }
        if (thresholdTimeElapsed[i] >= thresholdTimes[i])
        {
          thresholdActive[i] = false;
          StateMachineCOM.writeByte(i + 1);
        }
      }
    }
  }
  outputSyncLogic[1] = 1 - outputSyncLogic[1];
  dacValue.uint16[0] = EncoderPos * 16;
  dacValue.uint16[1] = outputSyncLogic[1] * logic3v3_bits;
  dacWrite();
#endif
}

void processPosition()
{
  if (wrappingEnabled)
  {
    switch (wrapMode)
    {
    case 0: // Bipolar mode
#if HARDWARE_VERSION == 1
      if (EncoderPos <= wrapPointInverse)
      {
        EncoderPos = wrapPoint;
        nWraps--;
      }
      else if (EncoderPos >= wrapPoint)
      {
        EncoderPos = wrapPointInverse;
        nWraps++;
      }
#else
      if (EncoderPos <= wrapPointInverse)
      {
        EncoderPos = wrapPoint + (wrapPointInverse - EncoderPos);
        hwEnc.write(EncoderPos);
        nWraps--;
      }
      else if (EncoderPos >= wrapPoint)
      {
        EncoderPos = wrapPointInverse + (EncoderPos - wrapPoint);
        hwEnc.write(EncoderPos);
        nWraps++;
      }
#endif
      break;
    case 1: // Unipolar mode
#if HARDWARE_VERSION == 1
      if (EncoderPos > wrapPoint)
      {
        EncoderPos = 0;
        nWraps++;
      }
      else if (EncoderPos < 0)
      {
        EncoderPos = wrapPoint;
        nWraps--;
      }
#else
      if (EncoderPos > wrapPoint)
      {
        EncoderPos = 0 + (wrapPoint - EncoderPos);
        hwEnc.write(EncoderPos);
        nWraps++;
      }
      else if (EncoderPos < 0)
      {
        EncoderPos = wrapPoint + (0 - EncoderPos);
        hwEnc.write(EncoderPos);
        nWraps--;
      }
#endif
      break;
    }
  }
  thisInd = iPositionBuffer[currentPositionBuffer];
  positionBuffer[thisInd][currentPositionBuffer] = EncoderPos;
  timeBuffer[thisInd][currentPositionBuffer] = currentTime;
  iPositionBuffer[currentPositionBuffer]++;
  positionBufferFlag = true;
}

void startLogging()
{
#if HARDWARE_VERSION == 1
  DataFile.seek(0);
  dataPos = 0;
  currentTime = 0;
  isLogging = true;
  iPositionBuffer[0] = 0;
  iPositionBuffer[1] = 0;
#endif
}

void logCurrentPosition()
{
#if HARDWARE_VERSION == 1
  if (nPositions > 0)
  {
    for (int i = 0; i < nPositions; i++)
    {
      sdWriteBuffer.int32[0] = positionBuffer[i][thisPositionBuffer];
      sdWriteBuffer.int32[1] = timeBuffer[i][thisPositionBuffer];
      DataFile.write(sdWriteBuffer.uint8, 8);
      dataPos += 1;
    }
  }
#endif
}

byte readByteFromSource(byte opSource)
{
  switch (opSource)
  {
  case 0:
    return myUSB.readByte();
    break;
  case 1:
    return StateMachineCOM.readByte();
    break;
#if HARDWARE_VERSION == 1
  case 2:
    return OutputStreamCOM.readByte();
    break;
#endif
  }
}

void resetDataStreams()
{
  usbStreaming = false;
  isLogging = false;
  dataPos = 0;
  EncoderPos = 0;
  nWraps = 0;
  iPositionBuffer[0] = 0;
  iPositionBuffer[1] = 0;
}
#if HARDWARE_VERSION == 2
void dacWrite()
{
  digitalWriteFast(DAC_CS_Pin, LOW);
  dacBuffer[0] = B00110000; // Channel
  dacBuffer[1] = dacValue.byteArray[1];
  dacBuffer[2] = dacValue.byteArray[0];
  SPI.transfer(dacBuffer, 3);
  digitalWriteFast(DAC_CS_Pin, HIGH);
  digitalWriteFast(DAC_CS_Pin, LOW);
  dacBuffer[0] = B00110001; // Channel
  dacBuffer[1] = dacValue.byteArray[3];
  dacBuffer[2] = dacValue.byteArray[2];
  SPI.transfer(dacBuffer, 3);
  digitalWriteFast(DAC_CS_Pin, HIGH);
}
void powerUpDAC()
{
  digitalWriteFast(DAC_CS_Pin, LOW);
  dacBuffer[0] = B01100000; // Enable internal reference
  dacBuffer[1] = 0;
  dacBuffer[2] = 0;
  SPI.transfer(dacBuffer, 3);
  digitalWriteFast(DAC_CS_Pin, HIGH);
}
#endif

#if HARDWARE_VERSION == 1
bool sdBusy()
{
  return ready ? SDcard.card()->isBusy() : false;
}
#endif

void returnModuleInfo()
{
  boolean fsmSupportsHwInfo = false;
  delayMicroseconds(100);
  if (StateMachineCOM.available() == 1)
  { // FSM firmware v23 or newer sends a second info request byte to indicate that it supports additional ops
    if (StateMachineCOM.readByte() == 255)
    {
      fsmSupportsHwInfo = true;
    }
  }
  StateMachineCOM.writeByte('A');                                     // Acknowledge
  StateMachineCOM.writeUint32(FirmwareVersion);                       // 4-byte firmware version
  StateMachineCOM.writeByte(sizeof(moduleName) - 1);                  // Length of module name
  StateMachineCOM.writeCharArray(moduleName, sizeof(moduleName) - 1); // Module name
  if (fsmSupportsHwInfo)
  {
    StateMachineCOM.writeByte(1);   // 1 if more info follows, 0 if not
    StateMachineCOM.writeByte('V'); // Op code for: Hardware major version
    StateMachineCOM.writeByte(HARDWARE_VERSION);
    StateMachineCOM.writeByte(1);   // 1 if more info follows, 0 if not
    StateMachineCOM.writeByte('v'); // Op code for: Hardware minor version
    StateMachineCOM.writeByte(circuitRevision);
  }
  StateMachineCOM.writeByte(0); // 1 if more info follows, 0 if not
}
