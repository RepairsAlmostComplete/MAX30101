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

  class Initialiser
  {
    uint8_t samp_avg;
    uint8_t fifo_rollover;
    uint8_t fifo_buff_full;
    char* mode_ctrl;
    uint8_t spo2_adc_range;
    uint8_t spo2_sample_rate;
    uint8_t led_pulse_width;
    char* mlslot1;
    char* mlslot2;
    char* mlslot3;
    char* mlslot4;
    
    public:
      //void SetVal(uint8_t, uint8_t, uint8_t, char*, uint8_t, uint8_t, uint8_t);
      //void SetVal(uint8_t, uint8_t, uint8_t, char*, uint8_t, uint8_t, uint8_t, char*[]);
      void SampAvg(uint8_t);
      void FIFORollover(uint8_t);
      void FIFOBuffFull(uint8_t);
      void ModeCtrl(char*);
      void SPO2ADCRange(uint8_t);
      void SPO2SampRate(uint8_t);
      void LEDPulseWidth(uint8_t);
      void MultiLEDSlot1(char*);
      void MultiLEDSlot2(char*);
      void MultiLEDSlot3(char*);
      void MultiLEDSlot4(char*);

      uint8_t SampAvg();
      uint8_t FIFORollover();
      uint8_t FIFOBuffFull();
      char* ModeCtrl();
      uint8_t SPO2ADCRange();
      uint8_t SPO2SampRate();
      uint8_t LEDPulseWidth();
      char* MultiLEDSlot1();
      char* MultiLEDSlot2();
      char* MultiLEDSlot3();
      char* MultiLEDSlot4();

      uint8_t SlotsInUse();
  };

  // Class to store the LED Data
  class FIFOData
  {
    uint32_t slot1;
    uint32_t slot2;
    uint32_t slot3;
    uint32_t slot4;

    public:
      void Slot1(uint32_t);
      void Slot2(uint32_t);
      void Slot3(uint32_t);
      void Slot4(uint32_t);

      uint32_t Slot1();
      uint32_t Slot2();
      uint32_t Slot3();
      uint32_t Slot4();
  };

  bool write_reg(uint8_t uch_addr, uint8_t uch_data);
  bool read_reg(uint8_t uch_addr, uint8_t *puch_data);
  bool initialise(Initialiser initOptions);
  bool reset();
  bool read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led, uint32_t *pun_green_led);
  uint8_t reg_fifo_config_val(uint8_t SMP_AVE, uint8_t FIFO_ROLLOVER_EN, uint8_t FIFO_A_FULL);
  uint8_t reg_mode_config_val(char* MODE_CTRL);
  byte reg_multi_led_mode(char* SLOT1, char* SLOT2);
  uint8_t reg_spo2_config_val(uint8_t SPO2_ADC_RGE, uint8_t SPO2_SR, uint8_t LED_PW);
  //uint8_t reg_prox_int_thresh_val(uint8_t prox_int_thresh);
}

#endif /* MAX30101_H */
