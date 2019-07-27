/*
 * File:   MAX30101.h
 * Author: Sally Longmore
 *
 * Created on 27 July 2019, 8:55z
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
  struct Initialiser
  {
    byte intEnable1; // Interrupt Enable 1 register value
    byte intEnable2; // Interrupt Enable 2 register value
    byte fifoConfig; // FIFO Configuration register value
    byte modeCtrl; // Mode Control register value
    byte spo2Config; // SpO2 Configuration register value
    byte mlSlot1; // Multi LED Slot 1 value
    byte mlSlot2; // Multi LED Slot 2 value
    byte mlSlot3; // Multi LED Slot 3 value
    byte mlSlot4; // Multi LED Slot 4 value
    byte ledPulseAmpRed; // LED Pulse Amplitude for the Red LED value
    byte ledPulseAmpIR; // LED Pulse Amplitude for the IR LED value
    byte ledPulseAmpGreen1; // LED Pulse Amplitude for the Green No. 1 LED value
    byte ledPulseAmpGreen2; // LED Pulse Amplitude for the Green No. 2 LED value
    byte ledPulseAmpPilot; // LED Pulse Amplitude for the Pilot LED value
    
    // <<< Interrupt Enable Registers >>>
    // -- Set the values --

    // Enables interrupt flag when data buffer is full
    void IntBuffFull(bool);

    // Enables 
    void IntPPGReady(bool);
    void IntAmbientLight(bool);
    void IntProximity(bool);
    void IntDieTempReady(bool);

    // -- FIFO Configuration Registers --
    
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

    // -- Mode Configuration Registers --
    // Set the mode control
    void ModeControl(char*);

    // -- SpO2 Configuration Registers --

    // Sets the SpO2 ADC Range
    void SPO2ADCRange(uint16_t);

    // Sets the SpO2 Sample Rate
    void SPO2SampRate(uint16_t);

    // Sets the LED Pulse Width
    void LEDPulseWidth(uint16_t);

    // -- LED Pulse Amplitude Registers --
    
    // Sets the LED Amplitude for the Red LED
    void LEDAmplitudeRED(uint8_t);

    // Sets the LED Amplitude for the IR LED
    void LEDAmplitudeIR(uint8_t);

    // Sets the LED Amplitude for the Green No.1 LED
    void LEDAmplitudeGREEN1(uint8_t);

    // Sets the LED Amplitude for the Green No.2 LED
    void LEDAmplitudeGREEN2(uint8_t);

    // Sets the LED Amplitude for the Pilot LED
    void LEDAmplitudePilot(uint8_t);

    // -- Multi-LED Mode Control Registers --
    
    // Sets the LED to use Multi LED Slot 1
    void MultiLEDSlot1(char*);

    // Sets the LED to use Multi LED Slot 2
    void MultiLEDSlot2(char*);

    // Sets the LED to use Multi LED Slot 3
    void MultiLEDSlot3(char*);

    // Sets the LED to use Multi LED Slot 4
    void MultiLEDSlot4(char*);

    // Return the values
    
    // Returns the Multi LED Control 1 register value
    byte MultiLEDCtrl1();

    // Returns the Multi LED Control 2 register value
    byte MultiLEDCtrl2();

    // Return Multi-LED Slots in Use
    uint8_t SlotsInUse();
      
  };

  struct InterruptStatus
  {
    bool fifoAlmostFull; // FIFO Data Buffer Almost Full flag
    bool fifoDataReady; // FIFO Data Available flag
    bool ambientLightOVF; // Ambient Light Affecting Sensor flag
    bool proximity; // Proximity flag
    bool powerReady; // Sensor Power Ready flag
    bool dieTempReady; // Die Temperature Conversion Ready flag

    public:
    
    // Polls the interrupt Status
    // Polling the interrupt status will clear all interrupt flags
    void CheckStatus();

  };

  class DieTempConversion
  {
    int8_t tempInt; // Holds the whole number of the temperature
    uint16_t tempFrac; // Holds the fraction number of the temperature

    public:
    
    // Request a Die Temperature Conversion from the sensor
    void Request();

    // Retrieve the Die Temperature from the sensor
    void Retrieve();

    // Return the whole number of the temperature
    int8_t GetWhole();

    // Return the fraction number of the temperature
    uint16_t GetFrac();

    // Return the temperature as a floating point integer
    float GetFloat();

    // Return the temperature as an integer shifted by four decimal places
    int32_t GetInt();
  };

  struct DataCounters
  {
    uint8_t writePtr; // Write Pointer - Write position in the data buffer
    uint8_t readPtr; // Read Pointer - Read position in the data buffer
    uint8_t overflowCtr; // Overflow Counter - The number of dropped data points
    int8_t dataAval = 0; // Number of data points available in the buffer to be read

    // Retrieve the data counters from the sensor
    void Retrieve();
  };

  /*
  * Struct to store the FIFO data options and pass them back to the calling function
  */
  struct FIFOData
  {
    uint32_t slot1; // Holds PPG data for the first LED
    uint32_t slot2; // Holds PPG data for the second LED (if in use)
    uint32_t slot3; // Holds PPG data for the third LED (if in use)
    uint32_t slot4; // Holds PPG data for the fourth LED (if in use)

    // Read data from the PPG Data Buffer
    bool ReadData();
  };

  bool WriteReg(uint8_t uch_addr, uint8_t uch_data);
  bool ReadReg(uint8_t uch_addr, uint8_t *puch_data);
  bool Initialise(Initialiser initOptions);
  bool Reset();
  byte LEDModeValues(char* LED_MODE);

}

#endif /* MAX30101_H */
