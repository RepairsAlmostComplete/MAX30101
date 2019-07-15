/*
 * File:   MAX30101.h
 * Author: Repairs Almost Complete
 *
 * Created on 11 July 2019, 5:57 PM
 */

//#include <arduino.h>
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
  bool write_reg(uint8_t uch_addr, uint8_t uch_data)
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
  bool read_reg(uint8_t uch_addr, uint8_t *puch_data)
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
  * Sets the Sampling Average initialisation option
  * Parameters:
  * - uint8_t value [Number of samples to average (1, 2, 3, 8, 16, 32)]
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::SampAvg(uint8_t value){
    switch (value) {
      case 1: // No averaging
        fifo_config += B00000000;
        break;
      case 2: // 2 samples averaged per FIFO sample
        fifo_config += B00100000;
        break;
      case 4: // 4 samples averaged per FIFO sample
        fifo_config += B01000000;
        break;
      case 8: // 8 samples averaged per FIFO sample
        fifo_config += B01100000;
        break;
      case 16: // 16 samples averaged per FIFO sample
        fifo_config += B10000000;
        break;
      case 32: // 32 samples averaged per FIFO sample``
        fifo_config += B10100000;
        break;
      default: // No averaging
        fifo_config += B00000000;
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
      fifo_config += B00000000; // FIFO Rolls on Full Disabled
    } else {
      fifo_config += B00010000; // FIFO Rolls on Full Enabled
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
    fifo_config += value;
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
    mode_ctrl = B00000000;

    if (mode == "HR"){
      mode_ctrl += MODE_HR;  // HR mode only
    } else if (mode == "SPO2") {
      mode_ctrl += MODE_SPO2;  // SPO2 mode only
    } else if (mode == "MULTI") {
      mode_ctrl += MODE_MULTI;  // Multi mode (Red, IR and Green)
    } else {
      mode_ctrl += MODE_SPO2; // Default to SPO2 mode
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
  void MAX30101::Initialiser::SPO2ADCRange(uint8_t value){
    switch (value){
      case 2048:
        spo2_config += B00000000; // LSB Size 7.81pA, Full Scale 2048nA
        break;
      case 4096:
        spo2_config += B00100000; // LSB Size 15.63pA, Full Scale 4096nA
        break;
      case 8192:
        spo2_config += B01000000; // LSB Size 31.25pA, Full Scale 8192nA
        break;
      case 16384:
        spo2_config += B01100000; // LSB Size 62.5pA, Full Scale 16384nA
        break;
      default:
        spo2_config += B00100000; // Default LSB Size 15.63pA, Full Scale 4096nA
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
  void MAX30101::Initialiser::SPO2SampRate(uint8_t value){
    switch (value){
      case 50:
        spo2_config += B00000000; // 50Hz (50 Samples per second)
        break;
      case 100:
        spo2_config += B00000100; // 100Hz (100 Samples per second)
        break;
      case 200:
        spo2_config += B00001000; // 200Hz (200 Samples per second)
        break;
      case 400:
        spo2_config += B00001100; // 400Hz (400 Samples per second)
        break;
      case 800:
        spo2_config += B00010000; // 800Hz (800 Samples per second)
        break;
      case 1000:
        spo2_config += B00010100; // 1000Hz (1000 Samples per second)
        break;
      case 1600:
        spo2_config += B00011000; // 1500Hz (1600 Samples per second)
        break;
      case 3200:
        spo2_config += B00011100; // 3200Hz (3200 Samples per second)
        break;
      default:
        spo2_config += B00000100; // 100Hz (100 Samples per second)
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
  void MAX30101::Initialiser::LEDPulseWidth(uint8_t value){
    switch (value) {
      case 15:
        spo2_config += B00000000; // 15 bits ADC Resolution with a pulse width of 69µs
        break;
      case 16:
        spo2_config += B00000001; // 16 bits ADC Resolution with a pulse width of 118µs
        break;
      case 17:
        spo2_config += B00000010; // 17 bits ADC Resolution with a pulse width of 215µs
        break;
      case 18:
        spo2_config += B00000011; // 18 bits ADC Resolution with a pulse width of 411µs
        break;
      default:
        spo2_config += B00000001; // 16 bits ADC Resolution with a pulse width of 118µs
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
    mlslot1 = led_mode_values(slot1);
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
    mlslot2 = led_mode_values(slot2);
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
    mlslot3 = led_mode_values(slot3);
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
    mlslot4 = led_mode_values(slot4);
  }

  /*
  * Sets the brightness of the Red LED
  * Parameters:
  * - uint8_t value [0.0 - 51.0 in 0.2 increments]
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::LEDAmplitudeRED(uint8_t value){
    led_pa_red = value / 0.2;
  }

  /*
  * Sets the brightness of the IR LED
  * Parameters:
  * - uint8_t value [0.0 - 51.0 in 0.2 increments]
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::LEDAmplitudeIR(uint8_t value){
    led_pa_ir = value / 0.2;
  }

  /*
  * Sets the brightness of the Green1 LED
  * Parameters:
  * - uint8_t value [0.0 - 51.0 in 0.2 increments]
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::LEDAmplitudeGREEN1(uint8_t value){
    led_pa_green1 = value / 0.2;
  }

  /*
  * Sets the brightness of the Green2 LED
  * Parameters:
  * - uint8_t value [0.0 - 51.0 in 0.2 increments]
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::LEDAmplitudeGREEN2(uint8_t value){
    led_pa_green2 = value / 0.2;
  }

  /*
  * Sets the brightness of the Pilot LED
  * Parameters:
  * - uint8_t value [0.0 - 51.0 in 0.2 increments]
  * Return value:
  * - void
  */
  void MAX30101::Initialiser::LEDAmplitudePilot(uint8_t value){
    led_pa_pilot = value / 0.2;
  }

  /*
  * Returns the FIFOConfig value for MAX30101 initialisation
  * Parameters:
  * - none
  * Return value:
  * - byte
  */
  byte MAX30101::Initialiser::FIFOConfig(){
    return fifo_config;
  }

  /*
  * Returns the ModeControl value for MAX30101 initialisation
  * Parameters:
  * - none
  * Return value:
  * - byte
  */
  byte MAX30101::Initialiser::ModeControl(){
    return mode_ctrl;
  }

  /*
  * Returns the SPO2Config value for MAX30101 initialisation
  * Parameters:
  * - none
  * Return value:
  * - byte
  */
  byte MAX30101::Initialiser::SPO2Config(){
    return spo2_config;
  }

  /*
  * Returns the MultiLEDCtrl1 value for MAX30101 initialisation
  * Parameters:
  * - none
  * Return value:
  * - byte
  */
  byte MAX30101::Initialiser::MultiLEDCtrl1(){
    byte multiLEDCtrl1 = mlslot1;
    
    // Add in the LED mode for the second slot
    multiLEDCtrl1 += mlslot2 << 4;

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
    byte multiLEDCtrl2 = mlslot3;
    
    // Add in the LED mode for the second slot
    multiLEDCtrl2 += mlslot4 << 4;

    return multiLEDCtrl2;
  }

  /*
  * Returns the LEDAmplitudeRED value for MAX30101 initialisation
  * Parameters:
  * - none
  * Return value:
  * - byte
  */
  byte MAX30101::Initialiser::LEDAmplitudeRED(){
    return led_pa_red;
  }

  /*
  * Returns the LEDAmplitudeIR value for MAX30101 initialisation
  * Parameters:
  * - none
  * Return value:
  * - byte
  */
  byte MAX30101::Initialiser::LEDAmplitudeIR(){
    return led_pa_ir;
  }

    /*
  * Returns the LEDAmplitudeGREEN1 value for MAX30101 initialisation
  * Parameters:
  * - none
  * Return value:
  * - byte
  */
  byte MAX30101::Initialiser::LEDAmplitudeGREEN1(){
    return led_pa_green1;
  }

  /*
  * Returns the LEDAmplitudeGREEN2 value for MAX30101 initialisation
  * Parameters:
  * - none
  * Return value:
  * - byte
  */
  byte MAX30101::Initialiser::LEDAmplitudeGREEN2(){
    return led_pa_green2;
  }

  /*
  * Returns the LEDAmplitudePilot value for MAX30101 initialisation
  * Parameters:
  * - none
  * Return value:
  * - byte
  */
  byte MAX30101::Initialiser::LEDAmplitudePilot(){
    return led_pa_pilot;
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

    if (mode_ctrl == MODE_HR){ // HR Mode: Only one slot is in use (Slot1: RED)
      slotsInUse = 1;
    }

    if (mode_ctrl == MODE_SPO2){ // SPO2 Mode: Two slots are in use (Slot1: RED, Slot2: IR)
      slotsInUse = 2;
    }

    if (mode_ctrl == MODE_MULTI){ // Multi Mode: Multi can be different configuration, so needs to be calculated
      if (mlslot1 != ML_DISABLED){
        slotsInUse ++;
      }

      if (mlslot2 != ML_DISABLED){
        slotsInUse ++;
      }

      if (mlslot3 != ML_DISABLED){
        slotsInUse ++;
      }

      if (mlslot4 != ML_DISABLED){
        slotsInUse ++;
      }
    }

    return slotsInUse;
  }
  // *** End of functions for Initialiser Class ***

  /*
  * Initialise the MAX30101 sensor
  * Parameters:
  * - Initialiser initOptions [Initialiser Object]
  * Return value:
  * - true on success
  * Initialises the MAX30101 sensor as per the passed in options via the Initialiser object.
  */
  bool initialise(Initialiser initOptions)
  {
    
    uint8_t partid; // Variable to store part ID check
    //Wire.begin(); // Setting up the Wire library to begin   - INIT OUTSIDE, or reset for SetClock feature -GL

    if(!MAX30101::write_reg(REG_MODE_CONFIG, 0x40)) //Reset device
      return false;
      delay(40);
    MAX30101::read_reg(REG_PART_ID, &partid);
    if(partid!=0x15)
      return false; // If wrong device, fail initialising - your code didn't check hence could succeed with nothing connected. -GL
    if(!MAX30101::write_reg(REG_MODE_CONFIG, initOptions.ModeControl())) // Values calculated from constants passed to the function (MODE_CTRL)
      return false;
    if(!MAX30101::write_reg(REG_MULTI_LED_CTRL1, initOptions.MultiLEDCtrl1()))
      return false;
    if(!MAX30101::write_reg(REG_MULTI_LED_CTRL2, initOptions.MultiLEDCtrl2()))
      return false;
    if(!MAX30101::write_reg(REG_FIFO_WR_PTR, 0x00)) //FIFO_WR_PTR[4:0] - Clearing the write pointer
      return false;
    if(!MAX30101::write_reg(REG_OVF_COUNTER, 0x00)) //OVF_COUNTER[4:0] - Clearing the overflow counter
      return false;
    if(!MAX30101::write_reg(REG_FIFO_RD_PTR, 0x00)) //FIFO_RD_PRT[4:0] - Clearing the read pointer
      return false;
    if(!MAX30101::write_reg(REG_FIFO_CONFIG, initOptions.FIFOConfig())) // Values calculated from constants passed to the function (SMP_AVE, FIFO_ROLLOVER_EN, FIFO_A_FULL)
      return false;
    if(!MAX30101::write_reg(REG_SPO2_CONFIG, initOptions.SPO2Config())) // Values calculated from constants passed to function (SPO2_ADC_RGE, SPO2_SR, LED_PW)
      return false;
    /*if(!MAX30101::write_reg(REG_LED1_PA, 0x24)) // Choose value for ~ 7mA for LED1 (0xFF for 50mA)
      return false;
    if(!MAX30101::write_reg(REG_LED2_PA, 0x24)) // Choose value for ~ 7mA for LED2
      return false;*/
    //if(!MAX30101::write_reg(REG_LED3_PA, 0x24)) // Choose value for ~ 7mA for LED2
    //  return false;
    //if(!MAX30101::write_reg(REG_LED4_PA, 0x24)) // Choose value for ~ 7mA for LED2
    //  return false;
    if(!MAX30101::write_reg(REG_LED1_PA, initOptions.LEDAmplitudeRED())) // Choose value for ~ 7mA for LED1 (0xFF for 50mA)
      return false;
    if(!MAX30101::write_reg(REG_LED2_PA, initOptions.LEDAmplitudeIR())) // Choose value for ~ 7mA for LED2
      return false;
    if(!MAX30101::write_reg(REG_LED3_PA, initOptions.LEDAmplitudeGREEN1())) // Choose value for ~ 7mA for LED2
      return false;
    if(!MAX30101::write_reg(REG_LED4_PA, initOptions.LEDAmplitudeGREEN2())) // Choose value for ~ 7mA for LED2
      return false;
    if(!MAX30101::write_reg(REG_PILOT_PA, initOptions.LEDAmplitudePilot())) // Choose value for ~ 25mA for Pilot LED
      return false;
    if(!MAX30101::write_reg(REG_PROX_INT_THRESH, 0x02))
      return false;
    if(!MAX30101::write_reg(REG_INTR_ENABLE_1, 0xF0)) //0xc0)) // Intr Setting Enabled for New Sample & Order Changed -GL
      return false;
    if(!MAX30101::write_reg(REG_INTR_ENABLE_2, 0x02))  // Should Interrupt on Temp Conversion -GL
      return false;
    slotsInUse = initOptions.SlotsInUse();
    return true; // You were missing this and hence made the function less useful! -GL
  }

  bool reset(){
    if(!MAX30101::write_reg(REG_MODE_CONFIG, 0x00))
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
  bool read_fifo(MAX30101::FIFOData &pun_Data) // FIFOData passed by reference, rather than pointer
  {

    pun_Data.slot1 = 0;
    pun_Data.slot2 = 0;
    pun_Data.slot3 = 0;
    pun_Data.slot4 = 0;

    Wire.beginTransmission(I2C_WRITE_ADDR);
    if(!Wire.write(byte(REG_FIFO_DATA)))
      return false;
    Wire.endTransmission();
    if(!Wire.requestFrom(I2C_WRITE_ADDR, 9))
      return false;

    if (slotsInUse > 0){ // Read data for the first LED slot
      pun_Data.slot1 |= (((Wire.read() &0x03) << 16) | (Wire.read() << 8) | Wire.read());
    }
    if (slotsInUse > 1){ // Read data for the second LED slot
      pun_Data.slot2 |= (((Wire.read()&0x03) << 16) | (Wire.read() << 8) | Wire.read()); // Compact notation for the above and below block, clearer and saves space. -GL
    }
    if (slotsInUse > 2){ // Read data for the third LED slot
      pun_Data.slot3 |= (((Wire.read()&0x03) << 16) | (Wire.read() << 8) | Wire.read());
    }
    if (slotsInUse > 3){ // Read data for the fourth LED slot
      pun_Data.slot4 |= (((Wire.read()&0x03) << 16) | (Wire.read() << 8) | Wire.read());
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
  byte led_mode_values(char* LED_MODE){
    
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
