/*
 * File:   MAX30101.h
 * Author: meimcounting
 *
 * Created on 25 July 2018, 5:57 PM
 */

//#include <arduino.h>
#include <Wire.h>
#include "max30101.h"

namespace MAX30101{
  
  /*
  * Write a value to the MAX30101 registers
  * Parameters:
  *   uch_addr    in    register Address
  *   uch_data    in    register data
  *
  * Return value:
  *   true on success
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
  *   uch_addr    in    register Address
  *   uch_data    out   register data
  *
  * Return value:
  *   true on success
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

  /*
  * Initialise the MAX30101 sensor
  * Parameters:
  *
  * Return value:
  *   true on success
  */
  bool initialise(uint8_t SMP_AVE, uint8_t FIFO_ROLLOVER_EN, uint8_t FIFO_A_FULL, char* MODE_CTRL, uint8_t SPO2_ADC_RGE, uint8_t SPO2_SR, uint8_t LED_PW, char* MULTI_LED_MODE[])
  {
    uint8_t partid; // Variable to store part ID check
    //Wire.begin(); // Setting up the Wire library to begin   - INIT OUTSIDE, or reset for SetClock feature -GL

    if(!MAX30101::write_reg(REG_MODE_CONFIG, 0x40)) //Reset device
      return false;
      delay(40);
    MAX30101::read_reg(REG_PART_ID, &partid);
    if(partid!=0x15)
      return false; // If wrong device, fail initialising - your code didn't check hence could succeed with nothing connected. -GL
    if(!MAX30101::write_reg(REG_MODE_CONFIG, reg_mode_config_val(MODE_CTRL))) // Values calculated from constants passed to the function (MODE_CTRL)
      return false;
    if(!MAX30101::write_reg(REG_MULTI_LED_CTRL1, reg_multi_led_mode(MULTI_LED_MODE[0], MULTI_LED_MODE[1])))
      return false;
    if(!MAX30101::write_reg(REG_MULTI_LED_CTRL2, reg_multi_led_mode(MULTI_LED_MODE[2], MULTI_LED_MODE[3])))
      return false;
    if(!MAX30101::write_reg(REG_FIFO_WR_PTR, 0x00)) //FIFO_WR_PTR[4:0] - Clearing the write pointer
      return false;
    if(!MAX30101::write_reg(REG_OVF_COUNTER, 0x00)) //OVF_COUNTER[4:0] - Clearing the overflow counter
      return false;
    if(!MAX30101::write_reg(REG_FIFO_RD_PTR, 0x00)) //FIFO_RD_PRT[4:0] - Clearing the read pointer
      return false;
    if(!MAX30101::write_reg(REG_FIFO_CONFIG, reg_fifo_config_val(SMP_AVE, FIFO_ROLLOVER_EN, FIFO_A_FULL))) // Values calculated from constants passed to the function (SMP_AVE, FIFO_ROLLOVER_EN, FIFO_A_FULL)
      return false;
    if(!MAX30101::write_reg(REG_SPO2_CONFIG, reg_spo2_config_val(SPO2_ADC_RGE, SPO2_SR, LED_PW))) // Values calculated from constants passed to function (SPO2_ADC_RGE, SPO2_SR, LED_PW)
      return false;
    if(!MAX30101::write_reg(REG_LED1_PA, 0x24)) // Choose value for ~ 7mA for LED1 (0xFF for 50mA)
      return false;
    if(!MAX30101::write_reg(REG_LED2_PA, 0x24)) // Choose value for ~ 7mA for LED2
      return false;
    if(!MAX30101::write_reg(REG_LED3_PA, 0x24)) // Choose value for ~ 7mA for LED2
      return false;
    if(!MAX30101::write_reg(REG_LED4_PA, 0x24)) // Choose value for ~ 7mA for LED2
      return false;
    if(!MAX30101::write_reg(REG_PILOT_PA, 0x7f)) // Choose value for ~ 25mA for Pilot LED
      return false;
    if(!MAX30101::write_reg(REG_PROX_INT_THRESH, 0x02))
      return false;
    if(!MAX30101::write_reg(REG_INTR_ENABLE_1, 0xF0)) //0xc0)) // Intr Setting Enabled for New Sample & Order Changed -GL
      return false;
    if(!MAX30101::write_reg(REG_INTR_ENABLE_2, 0x02))  // Should Interrupt on Temp Conversion -GL
      return false;
    return true; // You were missing this and hence made the function less useful! -GL
  }

  bool reset(){
    if(!MAX30101::write_reg(REG_MODE_CONFIG, 0x00))
      return false;
    return true;
  }

  /*
  * Read a value from the MAX30101 registers
  * Parameters:
  *   *pun_red_led   out   red led data pointer
  *   *pun_ir_led    out   ir led data pointer
  *
  * Return value:
  *   true on success
  */
  bool read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led, uint32_t *pun_green_led)
  {
//    uint32_t un_temp;
//    uint8_t uch_temp;
    *pun_ir_led = 0;
    *pun_red_led = 0;
    *pun_green_led = 0;
//    MAX30101::read_reg(REG_INTR_STATUS_1, &uch_temp);  // No idea why you're doing this? INT STATUS is read already, reading again may clear interrupts again!!! -GL
//    MAX30101::read_reg(REG_INTR_STATUS_2, &uch_temp);

    Wire.beginTransmission(I2C_WRITE_ADDR);
    if(!Wire.write(byte(REG_FIFO_DATA)))
      return false;
    Wire.endTransmission();
    if(!Wire.requestFrom(I2C_WRITE_ADDR, 9))
      return false;

/*    un_temp = Wire.read(); // Read the first byte
    un_temp <<= 16; // Bitshift the data to the left 16 places
    *pun_red_led += un_temp; // Add that data to pun_red_led
    un_temp = Wire.read(); // Read the next byte
    un_temp <<= 8; // Bitshift the data to the left 8 places
    *pun_red_led += un_temp; // Add that data to pun_red_led
    un_temp = Wire.read(); // Read the next byte
    *pun_red_led += un_temp; // Add that data to pun_red_led

    un_temp = Wire.read(); // Read the first byte
    un_temp <<= 16; // Bitshift the data to the left 16 places
    *pun_ir_led += un_temp; // Add that data to pun_ir_led
    un_temp = Wire.read(); // Read the next byte
    un_temp <<= 8; // Bitshift the data to the left 8 places
    *pun_ir_led += un_temp; // Add that data to pun_ir_led
    un_temp = Wire.read(); // Read the next byte
    *pun_ir_led += un_temp; // Add that data to pun_ir_led
*/
    *pun_red_led |= (((Wire.read() &0x03) << 16) | (Wire.read() << 8) | Wire.read());
    *pun_ir_led |= (((Wire.read()&0x03) << 16) | (Wire.read() << 8) | Wire.read()); // Compact notation for the above and below block, clearer and saves space. -GL
    *pun_green_led |= (((Wire.read()&0x03) << 16) | (Wire.read() << 8) | Wire.read());
    Wire.endTransmission();

//    *pun_red_led &= 0x03FFFF; // Apply mask MSB [23:18]
//    *pun_ir_led &= 0x03FFFF; // Apply mask MSB [23:18]

    return true;

  }

  /*
  * Configure the REG_FIFO_CONFIG parameters as a byte from supplied text
  * Parameters:
  *
  * Return value:
  *   true on success
  */
  uint8_t reg_fifo_config_val(uint8_t SMP_AVE, uint8_t FIFO_ROLLOVER_EN, uint8_t FIFO_A_FULL){

    byte b_temp;

    switch (SMP_AVE) {
      case 1: // No averaging
        b_temp = B00000000;
        break;
      case 2: // 2 samples averaged per FIFO sample
        b_temp = B00100000;
        break;
      case 4: // 4 samples averaged per FIFO sample
        b_temp = B01000000;
        break;
      case 8: // 8 samples averaged per FIFO sample
        b_temp = B01100000;
        break;
      case 16: // 16 samples averaged per FIFO sample
        b_temp = B10000000;
        break;
      case 32: // 32 samples averaged per FIFO sample``
        b_temp = B10100000;
        break;
      default: // No averaging
        b_temp = B00000000;
        break;
    }

    if (FIFO_ROLLOVER_EN == 0){
      b_temp += B00000000; // FIFO Rolls on Full Disabled
    } else {
      b_temp += B00010000; // FIFO Rolls on Full Enabled
    }

    switch (FIFO_A_FULL) { // FIFO Almost Full Value - Returns full when it has x unread samples
      case 32: // 32 unread samples (i.e. full FIFO) or 0 empty data slots
        b_temp += B00000000;
        break;
      case 31: // 31 unread samples or 1 empty data slots
        b_temp += B00000001;
        break;
      case 30: // 30 unread samples (2 empty data slots)
        b_temp += B00000010;
        break;
      case 29: // 29 unread samples (3 empty data slots)
        b_temp += B00000011;
        break;
      case 28: // 28 unread samples (4 empty data slots)
        b_temp += B00000100;
        break;
      case 27: // 27 unread samples (5 empty data slots)
        b_temp += B00000101;
        break;
      case 26: // 26 unread samples (6 empry data slots)
        b_temp += B00000110;
        break;
      case 25: // 25 unread samples (7 empty data slots)
        b_temp += B00000111;
        break;
      case 24: // 24 unread samples (8 empty data slots)
        b_temp += B00001000;
        break;
      case 23: // 23 unread samples (9 empty data slots)
        b_temp += B00001001;
        break;
      case 22: // 22 unread samples (10 empty data slots)
        b_temp += B00001010;
        break;
      case 21: // 21 unread samples (11 empty data slots)
        b_temp += B00001011;
        break;
      case 20: // 20 unread samples (12 empty data slots)
        b_temp += B00001100;
        break;
      case 19: // 19 unread samples (13 empty data slots)
        b_temp += B00001101;
        break;
      case 18: // 18 unread samples (14 empty data slots)
        b_temp += B00001110;
        break;
      case 17: // 17 unread samples (15 empty data slots)
        b_temp += B00001111;
        break;
      default: // 32 unread samples (0 emtpry data slots)
        b_temp += B00000000;
        break;
    }

    return b_temp;
  }

  uint8_t reg_mode_config_val(char* MODE_CTRL){
    byte b_temp;

    b_temp = B00000000;

    if (MODE_CTRL == "HR"){
      b_temp += B00000010;  // HR mode only
    } else if (MODE_CTRL == "SPO2") {
      b_temp += B00000011;  // SPO2 mode only
    } else if (MODE_CTRL == "MULTI") {
      b_temp += B00000111;  // Multi mode (Red, IR and Green)
    } else {
      b_temp += B00000011;
    }

    return b_temp;
  }

  // Returns the byte value for the LED mode requested
  byte led_mode_values(char* LED_MODE){
    
    if (LED_MODE == "DISABLED"){
      return B000;
    } else if (LED_MODE == "RED"){
      return B001;
    } else if (LED_MODE == "IR" ){
      return B010;
    } else if (LED_MODE == "GREEN"){
      return B011;
    } else if (LED_MODE = "NONE"){
      return B100;
    } else if (LED_MODE = "PILOT_RED"){
      return B101;
    } else if (LED_MODE = "PILOT_IR"){
      return B110;
    } else if (LED_MODE = "PILOT_RED"){
      return B111;
    }
    
    return B000;
  }

  // Used setup multi-LED mode, four slots over two registers, therefore two slots per register
  byte reg_multi_led_mode(char* SLOT1, char* SLOT2)
  {
    byte b_temp;

    b_temp = B00000000;

    // Get the LED mode and bit shift to the left four places for the first slot
    b_temp = led_mode_values(SLOT1);
    
    // Add in the LED mode for the second slot
    b_temp += led_mode_values(SLOT2) << 4;

    Serial.println();
    Serial.println(b_temp, BIN);    

    return b_temp;
  }

  uint8_t reg_spo2_config_val(uint8_t SPO2_ADC_RGE, uint8_t SPO2_SR, uint8_t LED_PW ){
    byte b_temp;

    switch (SPO2_ADC_RGE){
      case 2048:
        b_temp += B00000000; // LSB Size 7.81pA, Full Scale 2048nA
        break;
      case 4096:
        b_temp += B00100000; // LSB Size 15.63pA, Full Scale 4096nA
        break;
      case 8192:
        b_temp += B01000000; // LSB Size 31.25pA, Full Scale 8192nA
        break;
      case 16384:
        b_temp += B01100000; // LSB Size 62.5pA, Full Scale 16384nA
        break;
      default:
        b_temp += B00100000; // Default LSB Size 15.63pA, Full Scale 4096nA
        break;
    }

    switch (SPO2_SR){
      case 50:
        b_temp += B00000000; // 50Hz (50 Samples per second)
        break;
      case 100:
        b_temp += B00000100; // 100Hz (100 Samples per second)
        break;
      case 200:
        b_temp += B00001000; // 200Hz (200 Samples per second)
        break;
      case 400:
        b_temp += B00001100; // 400Hz (400 Samples per second)
        break;
      case 800:
        b_temp += B00010000; // 800Hz (800 Samples per second)
        break;
      case 1000:
        b_temp += B00010100; // 1000Hz (1000 Samples per second)
        break;
      case 1600:
        b_temp += B00011000; // 1500Hz (1600 Samples per second)
        break;
      case 3200:
        b_temp += B00011100; // 3200Hz (3200 Samples per second)
        break;
      default:
        b_temp += B00000100; // 100Hz (100 Samples per second)
        break;
    }

    switch (LED_PW) {
      case 15:
        b_temp += B00000000; // 15 bits ADC Resolution with a pulse width of 69µs
        break;
      case 16:
        b_temp += B00000001; // 16 bits ADC Resolution with a pulse width of 118µs
        break;
      case 17:
        b_temp += B00000010; // 17 bits ADC Resolution with a pulse width of 215µs
        break;
      case 18:
        b_temp += B00000011; // 18 bits ADC Resolution with a pulse width of 411µs
        break;
      default:
        b_temp += B00000001; // 16 bits ADC Resolution with a pulse width of 118µs
        break;
    }

    // ****** DEBUG CODE - REMOVE *******

    return b_temp;

  }


}
