/*
 * File:   MAX30101.h
 * Author: meimcounting
 *
 * Created on 25 July 2018, 5:57 PM
 */

#ifndef MAX30101_H
#define MAX30101_H

#include <arduino.h>
#include <Wire.h>

namespace MAX30101{
  // MAXREFDES117 Write and Read Address
  #define I2C_WRITE_ADDR 0x57 //0xAE
  #define I2C_READ_ADDR 0x58 //0xAF

  // MAXREFDES117 Register Addresses
  #define REG_INTR_STATUS_1 0x00 // Interrupt Status 1
  #define REG_INTR_STATUS_2 0x01 // Interrupt Status 2
  #define REG_INTR_ENABLE_1 0x02 // Interrupt Enable 1
  #define REG_INTR_ENABLE_2 0x03 // Interrupt Enable 2
  #define REG_FIFO_WR_PTR 0x04 // FIFO Write Pointer
  #define REG_OVF_COUNTER 0x05 // Overflow Counter
  #define REG_FIFO_RD_PTR 0x06 // FIFO Read Pointer
  #define REG_FIFO_DATA 0x07 // FIFO Data Register
  #define REG_FIFO_CONFIG 0x08 // FIFO Configuration
  #define REG_MODE_CONFIG 0x09 // Mode Configuration
  #define REG_SPO2_CONFIG 0x0A // SPO2 Configuration
  #define REG_LED1_PA 0x0C  // Red
  #define REG_LED2_PA 0x0D  // IR
  #define REG_LED3_PA 0x0E  // Green
  #define REG_LED4_PA 0x0F  // Green
  #define REG_PILOT_PA 0x10   // LED Pulse Proximity Mode (apparently not implemented in MAX30101, but still appears to work)
  #define REG_MULTI_LED_CTRL1 0x11 // Multi-LED Mode Control Registers
  #define REG_MULTI_LED_CTRL2 0x12 // Multi-LED Mode Control Registers
  #define REG_TEMP_INTR 0x1F  // Die Temp Integer
  #define REG_TEMP_FRAC 0x20 // Die Temp Fraction
  #define REG_TEMP_CONFIG 0x21 // Die Temp Config
  #define REG_PROX_INT_THRESH 0x30 // Not implemented
  #define REG_REV_ID 0xFE // Revision ID
  #define REG_PART_ID 0xFF // Part ID

  bool write_reg(uint8_t uch_addr, uint8_t uch_data);
  bool read_reg(uint8_t uch_addr, uint8_t *puch_data);
  bool initialise(uint8_t SMP_AVE, uint8_t FIFO_ROLLOVER_EN, uint8_t FIFO_A_FULL, char* MODE_CTRL, uint8_t SPO2_ADC_RGE, uint8_t SPO2_SR, uint8_t LED_PW, char* MULTI_LED_MODE[]);
  bool reset();
  bool read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led, uint32_t *pun_green_led);
  uint8_t reg_fifo_config_val(uint8_t SMP_AVE, uint8_t FIFO_ROLLOVER_EN, uint8_t FIFO_A_FULL);
  uint8_t reg_mode_config_val(char* MODE_CTRL);
  byte reg_multi_led_mode(char* SLOT1, char* SLOT2);
  uint8_t reg_spo2_config_val(uint8_t SPO2_ADC_RGE, uint8_t SPO2_SR, uint8_t LED_PW);
  //uint8_t reg_prox_int_thresh_val(uint8_t prox_int_thresh);
}

#endif /* MAX30101_H */
