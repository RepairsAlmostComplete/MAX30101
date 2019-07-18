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
  // MAX30101 Write and Read Address
  #define I2C_WRITE_ADDR 0x57 //0xAE
  #define I2C_READ_ADDR 0x58 //0xAF

  // MAX30101 Register Addresses
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
  #define REG_MULTI_LED_CTRL1 0x11 // Multi-LED Mode Control Registers for Slot 1 and Slot 2
  #define REG_MULTI_LED_CTRL2 0x12 // Multi-LED Mode Control Registers for Slot 3 and Slot 4
  #define REG_TEMP_INTR 0x1F  // Die Temp Integer
  #define REG_TEMP_FRAC 0x20 // Die Temp Fraction
  #define REG_TEMP_CONFIG 0x21 // Die Temp Config
  #define REG_PROX_INT_THRESH 0x30 // Not implemented
  #define REG_REV_ID 0xFE // Revision ID
  #define REG_PART_ID 0xFF // Part ID

  // MAX30101 Mode Control Types
  #define MODE_HR     B00000010 // Heart Rate only mode - uses red LED only
  #define MODE_SPO2   B00000011 // SPO2 mode - uses red and IR MultiLEDSlot1
  #define MODE_MULTI  B00000111 // Multi LED Mode - configurable LED use via reg_multi_led_mode

  // MAX30101 Multi LED Control Types
  #define ML_DISABLED     B000 // Slot is disabled
  #define ML_RED          B001 // Set slot to use Red LED
  #define ML_IR           B010 // Set slot to use IR LED
  #define ML_GREEN        B011 // Set slot to use Green LED
  #define ML_NONE         B100 // Set slot to no LED
  #define ML_PILOT_RED    B101 // Set slot to use Red LED as Pilot <<NOT IN USE>>
  #define ML_PILOT_IR     B110 // Set slot to use IR LED as Pilot <<NOT IN USE>>
  #define ML_PILOT_GREEN  B111 // Set slot to use Green LED as Pilot <<NOT IN USE>>

  // Macros
  #define BIT(n,i) (n>>i&1) // Macro to get a specific bit of an integer

  /*
  * Class that holds the MAX30101 initialise options
  */
  class Initialiser
  {
    byte int_enable1;
    byte int_enable2;
    byte fifo_config;
    byte mode_ctrl;
    byte spo2_config;
    byte mlslot1;
    byte mlslot2;
    byte mlslot3;
    byte mlslot4;
    byte led_pa_red;
    byte led_pa_ir;
    byte led_pa_green1;
    byte led_pa_green2;
    byte led_pa_pilot;
    
    public:
      // <<< Interrupt Enable Registers >>>
      // -- Set the values --

      // Enables interrupt flag when data buffer is full
      void IntBuffFull(bool);
      void IntPPGReady(bool);
      void IntAmbientLight(bool);
      void IntProximity(bool);
      void IntDieTempReady(bool);

      // Return the values
      byte InterruptEnabled1();
      byte InterruptEnabled2();

      // <<< FIFO Configuration Registers >>>
      // -- Set the values --

      // Sets the number of samples to average for a single data point
      // Valid values are 1, 2, 4, 8, 16, 32
      void SampAvg(uint8_t);

      // Enables overwrite of the buffer when full when true
      // true = rollover enabled
      // false = rollover disabled
      void FIFORollover(bool);

      // Sets the number of samples remaining in the buffer when the
      // FIFO full interrupt is flagged.
      // Valud values are 0 - 15 (samples remaining)
      void FIFOBuffFull(uint8_t);

      // Return the values

      byte FIFOConfig();

      // -- Mode Configuration Registers --
      // Set the values

      void ModeControl(char*);

      // Return the values

      byte ModeControl();

      // -- SpO2 Configuration Registers --
      // Set the values

      void SPO2ADCRange(uint16_t);
      void SPO2SampRate(uint16_t);
      void LEDPulseWidth(uint16_t);

      // Return the values

      byte SPO2Config();

      // -- LED Pulse Amplitude Registers --
      // Set the values

      void LEDAmplitudeRED(uint8_t);
      void LEDAmplitudeIR(uint8_t);
      void LEDAmplitudeGREEN1(uint8_t);
      void LEDAmplitudeGREEN2(uint8_t);
      void LEDAmplitudePilot(uint8_t);

      // Return the values

      byte LEDAmplitudeRED();
      byte LEDAmplitudeIR();
      byte LEDAmplitudeGREEN1();
      byte LEDAmplitudeGREEN2();
      byte LEDAmplitudePilot();

      // -- Multi-LED Mode Control Registers --
      // Set the values

      void MultiLEDSlot1(char*);
      void MultiLEDSlot2(char*);
      void MultiLEDSlot3(char*);
      void MultiLEDSlot4(char*);

      // Return the values
      
      byte MultiLEDCtrl1();
      byte MultiLEDCtrl2();

      // Return Multi-LED Slots in Use

      uint8_t SlotsInUse();
      
  };

  class InterruptStatus
  {
    bool fifo_almost_full;
    bool fifo_data_ready;
    bool ambient_light_ovf;
    bool proximity;
    bool power_ready;
    bool die_temp_ready;

    public:
    
    // Polls the interrupt Status
    // Polling the interrupt status will clear all interrupt flags
    void CheckStatus();

    bool FIFOAlmostFull();
    bool FIFODataReady();
    bool AmbientLightOverflow();
    bool Proximity();
    bool PowerReady();
    bool DieTempReady();

  };

  /*
  * Struct to store the FIFO data options and pass them back to the calling function
  */
  struct FIFOData
  {
    uint32_t slot1;
    uint32_t slot2;
    uint32_t slot3;
    uint32_t slot4;
  };

  bool write_reg(uint8_t uch_addr, uint8_t uch_data);
  bool read_reg(uint8_t uch_addr, uint8_t *puch_data);
  bool Initialise(Initialiser initOptions);
  bool reset();
  bool read_fifo(FIFOData &pun_Data);
  byte led_mode_values(char* LED_MODE);
  //uint8_t reg_prox_int_thresh_val(uint8_t prox_int_thresh);
}

#endif /* MAX30101_H */
