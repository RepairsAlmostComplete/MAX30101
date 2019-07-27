/*
 * File:   MAX30101.h
 * Author: Sally Longmore
 *
 * Created on 27 July 2019, 8:55z
 */

#include <Wire.h>
#include "max30101.h"

/*
* Namespace for MAX30101
*/
namespace MAX30101{ 

  uint8_t slotsInUse; // Stores the number of slots in use so that we know how many to process data for

  /*
  * Write a value to the MAX30101 registers
  * Parameters:
  * - uint8_t uch_addr [register Address]
  * - uint8_t uch_data [register data]
  * Return value:
  * - true on success
  */
  bool WriteReg(uint8_t uch_addr, uint8_t uch_data)
  {
    Wire.beginTransmission(I2C_WRITE_ADDR);
    int status = Wire.write(byte(uch_addr));
    if(!status)
      return status;
    status = Wire.write(byte(uch_data));
    if(!status)
      return false;
    Wire.endTransmission();
    return true;
  }

  /*
  * Read a value from the MAX30101 registers
  * Parameters:
  * - uint8_t uch_addr [register Address]
  * - uint8_t uch_data [register data]
  * Return value:
  * - true on success
  */
  bool ReadReg(uint8_t uch_addr, uint8_t *puch_data)
  {

      Wire.beginTransmission(I2C_WRITE_ADDR);
      if(!Wire.write(byte(uch_addr)))  // Move the pointer to the address to read
        return false;
      Wire.endTransmission();
      Wire.requestFrom(I2C_WRITE_ADDR, 8);

      *puch_data = Wire.read();

      Wire.endTransmission();
      return true; // Need return value -GL
  }

  // *** Start of functions for Initialiser Class ***

  /*
  * Sets the Interrupt Buffer Full initialisation option
  * Parameters:
  * - bool
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::IntBuffFull(bool value){
    if (value){
      intEnable1 |= (1 << 7);
    } else {
      intEnable1 |= (0 << 7);
    }
  }

  /*
  * Sets the Interrupt Buffer Full initialisation option
  * Parameters:
  * - bool
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::IntPPGReady(bool value){
    if (value){
      intEnable1 |= (1 << 6);
    } else {
      intEnable1 |= (0 << 6);
    }
  }

/*
  * Sets the Interrupt Buffer Full initialisation option
  * Parameters:
  * - bool
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::IntAmbientLight(bool value){
    if (value){
      intEnable1 |= (1 << 5);
    } else {
      intEnable1 |= (0 << 5);
    }
  }

  /*
  * Sets the Interrupt Buffer Full initialisation option
  * Parameters:
  * - bool
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::IntProximity(bool value){
    if (value){
      intEnable1 |= (1 << 4);
    } else {
      intEnable1 |= (0 << 4);
    }
  }

  /*
  * Sets the Interrupt Buffer Full initialisation option
  * Parameters:
  * - bool
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::IntDieTempReady(bool value){
    if (value){
      intEnable2 |= (1 << 1);
    } else {
      intEnable2 |= (0 << 1);
    }
  }

  /*
  * Sets the Sampling Average initialisation option
  * Parameters:
  * - uint8_t value [Number of samples to average (1, 2, 3, 8, 16, 32)]
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::SampAvg(uint8_t value){
    switch (value) {
      case 1: // No averaging
        fifoConfig += B00000000;
        break;
      case 2: // 2 samples averaged per FIFO sample
        fifoConfig += B00100000;
        break;
      case 4: // 4 samples averaged per FIFO sample
        fifoConfig += B01000000;
        break;
      case 8: // 8 samples averaged per FIFO sample
        fifoConfig += B01100000;
        break;
      case 16: // 16 samples averaged per FIFO sample
        fifoConfig += B10000000;
        break;
      case 32: // 32 samples averaged per FIFO sample``
        fifoConfig += B10100000;
        break;
      default: // No averaging
        fifoConfig += B00000000;
        break;
    }
  }

  /*
  * Sets the FIFO Rollover initialisation option
  * Parameters:
  * - bool value [true = on, false = off]
  * Return value:
  * - void
  * Indicates if the buffer should rollover if full.
  */
  void MAX30101::Initialiser::FIFORollover(bool value){
    if (value == true){
      fifoConfig += B00000000; // FIFO Rolls on Full Disabled
    } else {
      fifoConfig += B00010000; // FIFO Rolls on Full Enabled
    }
  }

  /*
  * Sets the FIFO Buffer Full initialisation option
  * Parameters:
  * - uint8_t value [0 - 15]
  * Return value:
  * - void
  * Sets the almost full flag at x samples free.
  */
  void MAX30101::Initialiser::FIFOBuffFull(uint8_t value){
    fifoConfig += value;
  }

  /*
  * Sets the Mode Control initialisation option
  * Parameters:
  * - char* mode [HR, SPO2, MULTI]
  * Return value:
  * - void
  * Sets the sensor mode to one of the following values:
  * - HR = Heart Rate mode, uses red LED only
  * - SPO2 = Pulse oximeter mode, uses red and IR LEDs
  * - MULTI = Multi led mode, configurable via MultiLEDCtrl1/2
  */
  void MAX30101::Initialiser::ModeControl(char* mode){
    modeCtrl = B00000000;

    if (mode == "HR"){
      modeCtrl += MODE_HR;  // HR mode only
    } else if (mode == "SPO2") {
      modeCtrl += MODE_SPO2;  // SPO2 mode only
    } else if (mode == "MULTI") {
      modeCtrl += MODE_MULTI;  // Multi mode (Red, IR and Green)
    } else {
      modeCtrl += MODE_SPO2; // Default to SPO2 mode
    }
  }

  /*
  * Sets the SPO2 ADC Range initialisation option
  * Parameters:
  * - uint8_t value [2048, 4096, 8192, 16384]
  * Return value:
  * - void
  * Sets the ADC range in bits.
  */
  void MAX30101::Initialiser::SPO2ADCRange(uint16_t value){
    switch (value){
      case 2048:
        spo2Config += B00000000; // LSB Size 7.81pA, Full Scale 2048nA
        break;
      case 4096:
        spo2Config += B00100000; // LSB Size 15.63pA, Full Scale 4096nA
        break;
      case 8192:
        spo2Config += B01000000; // LSB Size 31.25pA, Full Scale 8192nA
        break;
      case 16384:
        spo2Config += B01100000; // LSB Size 62.5pA, Full Scale 16384nA
        break;
      default:
        spo2Config += B00100000; // Default LSB Size 15.63pA, Full Scale 4096nA
        break;
    }
  }

  /*
  * Sets the Sample Rate initialisation option
  * Parameters:
  * - uint8_t value [50, 100, 200, 400, 800, 1000, 1600, 3200]
  * Return value:
  * - void
  * Sets the sample rate in Hz.
  * NOTE: Higher sample rates requre the pulse width to be changed.
  * See documentation for more information.
  */
  void MAX30101::Initialiser::SPO2SampRate(uint16_t value){
    switch (value){
      case 50:
        spo2Config += B00000000; // 50Hz (50 Samples per second)
        break;
      case 100:
        spo2Config += B00000100; // 100Hz (100 Samples per second)
        break;
      case 200:
        spo2Config += B00001000; // 200Hz (200 Samples per second)
        break;
      case 400:
        spo2Config += B00001100; // 400Hz (400 Samples per second)
        break;
      case 800:
        spo2Config += B00010000; // 800Hz (800 Samples per second)
        break;
      case 1000:
        spo2Config += B00010100; // 1000Hz (1000 Samples per second)
        break;
      case 1600:
        spo2Config += B00011000; // 1500Hz (1600 Samples per second)
        break;
      case 3200:
        spo2Config += B00011100; // 3200Hz (3200 Samples per second)
        break;
      default:
        spo2Config += B00000100; // 100Hz (100 Samples per second)
        break;
    }
  }

  /*
  * Sets the LED Pulse Width initialisation option
  * Parameters:
  * - uint8_t value [69, 118, 215, 411]
  * Return value:
  * - void
  * Sets the pulse width in µs.
  * NOTE: Also indirectly sets the ADC Resolution
  * - 69µs = 15 bits
  * - 118µs = 16 bits
  * - 215µs = 17 bits
  * - 411µs = 18 bits
  */
  void MAX30101::Initialiser::LEDPulseWidth(uint16_t value){
    switch (value) {
      case 15:
        spo2Config += B00000000; // 15 bits ADC Resolution with a pulse width of 69µs
        break;
      case 16:
        spo2Config += B00000001; // 16 bits ADC Resolution with a pulse width of 118µs
        break;
      case 17:
        spo2Config += B00000010; // 17 bits ADC Resolution with a pulse width of 215µs
        break;
      case 18:
        spo2Config += B00000011; // 18 bits ADC Resolution with a pulse width of 411µs
        break;
      default:
        spo2Config += B00000001; // 16 bits ADC Resolution with a pulse width of 118µs
        break;
    }
  }

  /*
  * Selects the LED for slot 1 in Multi LED mode
  * Parameters:
  * - uint8_t slot1 [RED, IR, GREEN, DISABLED]
  * Return value:
  * - void
  * Not required where HR and SPO2 selected for Mode Control.
  * NOTE: Slots should be filled from slot1 to slot4, disabled
  * slots should be at the end of available slots.
  */
  void MAX30101::Initialiser::MultiLEDSlot1(char* slot1){
    mlSlot1 = LEDModeValues(slot1);
  }

  /*
  * Selects the LED for slot 2 in Multi LED mode
  * Parameters:
  * - uint8_t slot2 [RED, IR, GREEN, DISABLED]
  * Return value:
  * - void
  * Not required where HR and SPO2 selected for Mode Control.
  * NOTE: Slots should be filled from slot1 to slot4, disabled
  * slots should be at the end of available slots.
  */
  void MAX30101::Initialiser::MultiLEDSlot2(char* slot2){
    mlSlot2 = LEDModeValues(slot2);
  }

  /*
  * Selects the LED for slot 3 in Multi LED mode
  * Parameters:
  * - uint8_t slot3 [RED, IR, GREEN, DISABLED]
  * Return value:
  * - void
  * Not required where HR and SPO2 selected for Mode Control.
  * NOTE: Slots should be filled from slot1 to slot4, disabled
  * slots should be at the end of available slots.
  */
  void MAX30101::Initialiser::MultiLEDSlot3(char* slot3){
    mlSlot3 = LEDModeValues(slot3);
  }

  /*
  * Selects the LED for slot 4 in Multi LED mode
  * Parameters:
  * - uint8_t slot4 [RED, IR, GREEN, DISABLED]
  * Return value:
  * - void
  * Not required where HR and SPO2 selected for Mode Control.
  * NOTE: Slots should be filled from slot1 to slot4, disabled
  * slots should be at the end of available slots.
  */
  void MAX30101::Initialiser::MultiLEDSlot4(char* slot4){
    mlSlot4 = LEDModeValues(slot4);
  }

  /*
  * Sets the brightness of the Red LED
  * Parameters:
  * - uint8_t value [0.0 - 51.0 in 0.2 increments]
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::LEDAmplitudeRED(uint8_t value){
    ledPulseAmpRed = value / 0.2;
  }

  /*
  * Sets the brightness of the IR LED
  * Parameters:
  * - uint8_t value [0.0 - 51.0 in 0.2 increments]
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::LEDAmplitudeIR(uint8_t value){
    ledPulseAmpIR = value / 0.2;
  }

  /*
  * Sets the brightness of the Green1 LED
  * Parameters:
  * - uint8_t value [0.0 - 51.0 in 0.2 increments]
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::LEDAmplitudeGREEN1(uint8_t value){
    ledPulseAmpGreen1 = value / 0.2;
  }

  /*
  * Sets the brightness of the Green2 LED
  * Parameters:
  * - uint8_t value [0.0 - 51.0 in 0.2 increments]
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::LEDAmplitudeGREEN2(uint8_t value){
    ledPulseAmpGreen2 = value / 0.2;
  }

  /*
  * Sets the brightness of the Pilot LED
  * Parameters:
  * - uint8_t value [0.0 - 51.0 in 0.2 increments]
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::LEDAmplitudePilot(uint8_t value){
    ledPulseAmpPilot = value / 0.2;
  }

  /*
  * Returns the MultiLEDCtrl1 value for MAX30101 initialisation
  * Parameters:
  * - none
  * Return value:
  * - byte
  */
  byte MAX30101::Initialiser::MultiLEDCtrl1(){
    byte multiLEDCtrl1 = mlSlot1;
    
    // Add in the LED mode for the second slot
    multiLEDCtrl1 += mlSlot2 << 4;

    return multiLEDCtrl1;
  }

  /*
  * Returns the MultiLEDCtrl2 value for MAX30101 initialisation
  * Parameters:
  * - none
  * Return value:
  * - byte
  */
  byte MAX30101::Initialiser::MultiLEDCtrl2(){
    byte multiLEDCtrl2 = mlSlot3;
    
    // Add in the LED mode for the second slot
    multiLEDCtrl2 += mlSlot4 << 4;

    return multiLEDCtrl2;
  }

  /*
  * Returns the number of Multi LED Control slots in use
  * Parameters:
  * - none
  * Return value:
  * - unit8_t
  */
  uint8_t MAX30101::Initialiser::SlotsInUse(){ // Required so that we know how much data to collect
    uint8_t slotsInUse = 0;

    if (modeCtrl == MODE_HR){ // HR Mode: Only one slot is in use (Slot1: RED)
      slotsInUse = 1;
    }

    if (modeCtrl == MODE_SPO2){ // SPO2 Mode: Two slots are in use (Slot1: RED, Slot2: IR)
      slotsInUse = 2;
    }

    if (modeCtrl == MODE_MULTI){ // Multi Mode: Multi can be different configuration, so needs to be calculated
      if (mlSlot1 != ML_DISABLED){
        slotsInUse ++;
      }

      if (mlSlot2 != ML_DISABLED){
        slotsInUse ++;
      }

      if (mlSlot3 != ML_DISABLED){
        slotsInUse ++;
      }

      if (mlSlot4 != ML_DISABLED){
        slotsInUse ++;
      }
    }

    return slotsInUse;
  }
  // *** End of functions for Initialiser Class ***

  // *** Start of Functions for InterruptStatus Class ***

  /*
  * Initiates a poll of the sensor for the status of all interrupt flags
  * Parameters:
  * - none
  * Return value:
  * - void
  */
  void MAX30101::InterruptStatus::CheckStatus(){
    uint8_t intStatus1;
    uint8_t intStatus2;

    fifoAlmostFull = false;
    fifoDataReady = false;
    ambientLightOVF = false;
    proximity = false;
    powerReady = false;
    dieTempReady = false;

    Wire.beginTransmission(I2C_WRITE_ADDR);
    Wire.write(REG_INTR_STATUS_1);
    Wire.endTransmission();
    Wire.requestFrom(I2C_WRITE_ADDR, 2);
    intStatus1 = Wire.read();
    intStatus2 = Wire.read();
    Wire.endTransmission();

    if (BIT(intStatus1, 7) == 1){
      fifoAlmostFull = true;
    }

    if (BIT(intStatus1, 6) == 1){
      fifoDataReady = true;
    }

    if (BIT(intStatus1, 5) == 1){
      ambientLightOVF = true;
    }
    
    if (BIT(intStatus1, 4) == 1){
      proximity = true;
    }

    if (BIT(intStatus1, 0) == 1){
      powerReady = true;
    }

    if (BIT(intStatus2, 1) == 1){
      dieTempReady = true;
    }

  }

  // *** End of functions for Interrupt Class ***

  // *** Start of functions for DieTempConversion Class ***

  /*
  * Request a Die Temperature Conversion from the sensor
  * Parameters:
  * - none
  * Return value:
  * - void
  */
  void MAX30101::DieTempConversion::Request(){
    MAX30101::WriteReg(REG_TEMP_CONFIG, 0x01);
  }

  /*
  * Retreive a Die Temperature Conversion from the sensor
  * Parameters:
  * - none
  * Return value:
  * - void
  */
  void MAX30101::DieTempConversion::Retrieve(){
    uint8_t rawInt;
    uint8_t rawFrac;
    
    MAX30101::ReadReg(REG_TEMP_INTR, &rawInt);
    MAX30101::ReadReg(REG_TEMP_FRAC, &rawFrac);

    if (rawInt <= 127){
      tempInt = rawInt;
    } else {
      tempInt = -(256 - rawInt);
    }

    tempFrac = rawFrac * 625;
  }

  /*
  * Returns the Die Temperature as a floating point integer
  * Parameters:
  * - none
  * Return value:
  * - float
  */
  float MAX30101::DieTempConversion::GetFloat(){
    float tempFloat = tempFrac;
    tempFloat = tempInt + tempFloat / 10000;

    return (tempFloat);
  }

  /*
  * Returns the Die Temperature as an unsigned integer
  * shifted by four decimal places (i.e. x10000)
  * Parameters:
  * - none
  * Return value:
  * - int32_t
  */
  int32_t MAX30101::DieTempConversion::GetInt(){
    return (tempInt * 10000 + tempFrac);
  }

  /*
  * Returns the whole number portion of the Die Temperature
  * Parameters:
  * - none
  * Return value:
  * - int8_t
  */
  int8_t MAX30101::DieTempConversion::GetWhole(){
    return (tempInt);
  }

  /*
  * Returns the fraction portion of the Die Temperature
  * Parameters:
  * - none
  * Return value:
  * - uint16_t
  */
  uint16_t MAX30101::DieTempConversion::GetFrac(){
    return (tempFrac);
  }

  // *** End of functions for DieTempConversion Class ***

  // *** Start of functions for DataCounters Class ***

  /*
  * Retreive the Data Counters from the sensor
  * Parameters:
  * - none
  * Return value:
  * - void
  */
  void MAX30101::DataCounters::Retrieve(){
    Wire.beginTransmission(I2C_WRITE_ADDR);
    Wire.write(REG_FIFO_WR_PTR);
    Wire.endTransmission();
    Wire.requestFrom(I2C_WRITE_ADDR, 3);
    writePtr = Wire.read();
    overflowCtr = Wire.read();
    readPtr = Wire.read();
    Wire.endTransmission();

    if (readPtr <= writePtr) {
        dataAval = writePtr - readPtr;
      } else {
        dataAval = 32 - readPtr + writePtr;
      }
  }

  // *** End of functions for DataCounters Class ***

  /*
  * Initialise the MAX30101 sensor
  * Parameters:
  * - Initialiser initOptions [Initialiser Object]
  * Return value:
  * - true on success
  * Initialises the MAX30101 sensor as per the passed in options via the Initialiser object.
  */
  bool Initialise(Initialiser initOptions)
  {
    
    uint8_t partid; // Variable to store part ID check    

    if(!MAX30101::WriteReg(REG_MODE_CONFIG, 0x40)) //Reset device
      return false;
      delay(40);
    MAX30101::ReadReg(REG_PART_ID, &partid);
    if(partid!=0x15)
      return false; // If wrong device, fail initialising
    if(!MAX30101::WriteReg(REG_MODE_CONFIG, initOptions.modeCtrl)) // Values calculated from constants passed to the function (MODE_CTRL)
      return false;
    if(!MAX30101::WriteReg(REG_MULTI_LED_CTRL1, initOptions.MultiLEDCtrl1()))
      return false;
    if(!MAX30101::WriteReg(REG_MULTI_LED_CTRL2, initOptions.MultiLEDCtrl2()))
      return false;
    if(!MAX30101::WriteReg(REG_FIFO_WR_PTR, 0x00)) //FIFO_WR_PTR[4:0] - Clearing the write pointer
      return false;
    if(!MAX30101::WriteReg(REG_OVF_COUNTER, 0x00)) //OVF_COUNTER[4:0] - Clearing the overflow counter
      return false;
    if(!MAX30101::WriteReg(REG_FIFO_RD_PTR, 0x00)) //FIFO_RD_PRT[4:0] - Clearing the read pointer
      return false;
    if(!MAX30101::WriteReg(REG_FIFO_CONFIG, initOptions.fifoConfig)) // Values calculated from constants passed to the function (SMP_AVE, FIFO_ROLLOVER_EN, FIFO_A_FULL)
      return false;
    if(!MAX30101::WriteReg(REG_SPO2_CONFIG, initOptions.spo2Config)) // Values calculated from constants passed to function (SPO2_ADC_RGE, SPO2_SR, LED_PW)
      return false;
    if(!MAX30101::WriteReg(REG_LED1_PA, initOptions.ledPulseAmpRed)) // Choose value for ~ 7mA for LED1 (0xFF for 50mA)
      return false;
    if(!MAX30101::WriteReg(REG_LED2_PA, initOptions.ledPulseAmpIR)) // Choose value for ~ 7mA for LED2
      return false;
    if(!MAX30101::WriteReg(REG_LED3_PA, initOptions.ledPulseAmpGreen1)) // Choose value for ~ 7mA for LED2
      return false;
    if(!MAX30101::WriteReg(REG_LED4_PA, initOptions.ledPulseAmpGreen2)) // Choose value for ~ 7mA for LED2
      return false;
    if(!MAX30101::WriteReg(REG_PILOT_PA, initOptions.ledPulseAmpPilot)) // Choose value for ~ 25mA for Pilot LED
      return false;
    if(!MAX30101::WriteReg(REG_PROX_INT_THRESH, 0x02))
      return false;
    if(!MAX30101::WriteReg(REG_INTR_ENABLE_1, initOptions.intEnable1)) //0xc0)) // Intr Setting Enabled for New Sample & Order Changed
      return false;
    if(!MAX30101::WriteReg(REG_INTR_ENABLE_2, initOptions.intEnable2))  // Should Interrupt on Temp Conversion
      return false;
    slotsInUse = initOptions.SlotsInUse();
    return true;
  }

  bool reset(){
    if(!MAX30101::WriteReg(REG_MODE_CONFIG, 0x00))
      return false;
    return true;
  }

  /*
  * Read data from the MAX30101 FIFO buffer register
  * Parameters:
  * - FIFOData  &pun_Data [FIFOData Object]
  * Return value:
  * - bool [true on success]
  */
  bool MAX30101::FIFOData::ReadData() // FIFOData passed by reference, rather than pointer
  {

    slot1 = 0;
    slot2 = 0;
    slot3 = 0;
    slot4 = 0;

    Wire.beginTransmission(I2C_WRITE_ADDR);
    if(!Wire.write(byte(REG_FIFO_DATA)))
      return false;
    Wire.endTransmission();
    if(!Wire.requestFrom(I2C_WRITE_ADDR, 9))
      return false;

    if (slotsInUse > 0){ // Read data for the first LED slot
      slot1 |= (((Wire.read() &0x03) << 16) | (Wire.read() << 8) | Wire.read());
    }
    if (slotsInUse > 1){ // Read data for the second LED slot
      slot2 |= (((Wire.read()&0x03) << 16) | (Wire.read() << 8) | Wire.read()); // Compact notation for the above and below block, clearer and saves space. -GL
    }
    if (slotsInUse > 2){ // Read data for the third LED slot
      slot3 |= (((Wire.read()&0x03) << 16) | (Wire.read() << 8) | Wire.read());
    }
    if (slotsInUse > 3){ // Read data for the fourth LED slot
      slot4 |= (((Wire.read()&0x03) << 16) | (Wire.read() << 8) | Wire.read());
    }
    
    Wire.endTransmission();

    return true;

  }

  /*
  * Returns the byte value for the requested LED mode
  * Parameters:
  * - char* LED_MODE [LED Mode to set]
  * Return value:
  * - byte
  */
  byte LEDModeValues(char* LED_MODE){
    
    if (LED_MODE == "DISABLED"){
      return ML_DISABLED;
    } else if (LED_MODE == "RED"){
      return ML_RED;
    } else if (LED_MODE == "IR" ){
      return ML_IR;
    } else if (LED_MODE == "GREEN"){
      return ML_GREEN;
    } else if (LED_MODE = "NONE"){
      return ML_NONE;
    } else if (LED_MODE = "PILOT_RED"){
      return ML_PILOT_RED;
    } else if (LED_MODE = "PILOT_IR"){
      return ML_PILOT_IR;
    } else if (LED_MODE = "PILOT_GREEN"){
      return ML_PILOT_GREEN;
    }
    
    return B000;
  }

}
