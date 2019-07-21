#include <Wire.h>
#include "max30101.h"

#define INTBUFFFULL true    // Defines if the Buffer Full Interrupt Flag should be enabled or not
#define INTPPGREADY true    // Defines if the PPG Ready Interrupt Flag should be enabled or not
#define INTAMBLIGHT true    // Defines if the Ambient Light Cancellation Overflow Interrupt Flag sould be enabled or not
#define INTPROXIMITY true   // Defines if the Proximity Threshold Triggered Interrupt Flag is enabled or not
#define INTDIETEMP true     // Defines it the Internal Die Temperature Interrupt Flag is enabled or not
#define SAMP_AVE 1          // Sampling average (Pos Val: 1, 2, 4, 8, 16, 32)
#define FIFO_RO true        // Roll over on full (false = OFF, true = ON)
#define ALMOST_FULL 8      // Set almost full flag at x samples free (Pos Val: 0 - 15)
#define SEN_MODE "MULTI"     // Set the sensor mode (Pos Val: HR, SPO2, MULTI)
#define ADC_RANGE 4096      // SPO2 ADC range control (Pos Val: 2048, 4096, 8192, 16384)
#define SAMP_RATE 100       // Sampling rate in Hz (Pos Val: 50, 100, 200, 400, 800, 1000, 1600, 3200)
#define LED_PULSE_WIDTH 411 // LED Pulse Width, also indirectly sets the ADC resolution (Pos Val: 69µs (15 bits), 118µs (16 bits), 215µs (17 bits), 411µs (18 bits))
#define MULTI_LED_SLOT1 "RED" // The led to be used with slot 1 in Multi LED mode
#define MULTI_LED_SLOT2 "IR" // The led to be used with slot 2 in Multi LED mode
#define MULTI_LED_SLOT3 "GREEN" // The led to be used with slot 3 in Multi LED mode
#define MULTI_LED_SLOT4 "DISABLED" // The led to be used with slot 4 in Multi LED mode, set to DISABLED when not in use
#define LED_PA_RED 7.2 // (7.2) Sets the Pulse Amplitude (brightness) of the RED LED in mA (0.0mA - 51.0mA in 0.2mA increments) Orig CFG 7.2
#define LED_PA_IR 7.2 // (7.2) Sets the Pulse Amplitude (brightness) of the IR LED in mA (0.0mA - 51.0mA in 0.2mA increments) Orig CFG 7.2
#define LED_PA_GREEN1 100 // (51) Sets the Pulse Amplitude (brightness) of the GREEN1 LED in mA (0.0mA - 51.0mA in 0.2mA increments) Orig CFG 7.2
#define LED_PA_GREEN2 100 // (51) Sets the Pulse Amplitude (brightness) of the GREEN2 LED in mA (0.0mA - 51.0mA in 0.2mA increments) Orig CFG 7.2
#define LED_PA_PILOT 255 // (127) Sets the Pulse Amplitude (brightness) of the PILOT LED in mA (0.0mA - 51.0mA in 0.2mA increments) Orig CFG 7.2

#define BIT(n,i) (n>>i&1) // Macro to get a specific bit of an integer

//#define I2C_MULTIPLEX_ADDR 0x70 // MUX I2C Address // Not Required as we are using a single sensor
#define NRCHAN 8
// Number of Active Channels
// #define RSTPIN 0            // Multiplex RST pin

unsigned long lastAnalog; // records last time analog-channel sampled

uint32_t sampleNo[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint32_t sampleTime[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t startLogging[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint32_t analogSampNo = 0;

void setup()
{
    Serial.begin(921600);
    //Serial.begin(256000);

    while(!Serial.available()){

    }

    //Begin I2C
    //Wire.begin(5, 4);
    Wire.begin();
    Wire.setClock(400000);

    Serial.println();
 
    MAX30101::Initialiser initOptions;
    initOptions.IntBuffFull(INTBUFFFULL);
    initOptions.IntPPGReady(INTPPGREADY);
    initOptions.IntAmbientLight(INTAMBLIGHT);
    initOptions.IntProximity(INTPROXIMITY);
    initOptions.IntDieTempReady(INTDIETEMP);
    initOptions.SampAvg(SAMP_AVE);
    initOptions.FIFORollover(FIFO_RO);
    initOptions.FIFOBuffFull(ALMOST_FULL);
    initOptions.ModeControl(SEN_MODE);
    initOptions.SPO2ADCRange(ADC_RANGE);
    initOptions.SPO2SampRate(SAMP_RATE);
    initOptions.LEDPulseWidth(LED_PULSE_WIDTH);
    initOptions.LEDAmplitudeRED(LED_PA_RED);
    initOptions.LEDAmplitudeIR(LED_PA_IR);
    initOptions.LEDAmplitudeGREEN1(LED_PA_GREEN1);
    initOptions.LEDAmplitudeGREEN2(LED_PA_GREEN2);
    initOptions.MultiLEDSlot1(MULTI_LED_SLOT1);
    initOptions.MultiLEDSlot2(MULTI_LED_SLOT2);
    initOptions.MultiLEDSlot3(MULTI_LED_SLOT3);
    initOptions.MultiLEDSlot4(MULTI_LED_SLOT4);
    initOptions.LEDAmplitudePilot(LED_PA_PILOT);

    uint8_t a = 15;
    byte b = a;

    // Setup MAX PPG Sensor
    Serial.print("Initialising PPG Sensor.... ");
    while (!MAX30101::Initialise(initOptions)) {
      Serial.println("Failed, retrying ...");
      delay(1000);
    }
    //MAX30101::write_reg(REG_TEMP_CONFIG, 0x01); // Initiates a temperature conversion
    MAX30101::DieTempConvRequest(); // Initiate a temperature conversion
    Serial.println("Complete");

    // Setup ADC
    pinMode(A0, INPUT);
    lastAnalog = millis();

}

void loop()
{
    uint8_t data2;
    int c = 0;
    // Using a double read to get both INTR STATUS as the above is invalid
    // Chip appears to reset REG_INTR_STATUS_2 on reading REG_INTR_STATUS_1 and vice versa
    /*/Wire.beginTransmission(0x57);
    Wire.write(REG_INTR_STATUS_2);
    Wire.endTransmission();
    Wire.requestFrom(0x57, 1);
    //data = Wire.read();
    data2 = Wire.read();
    Wire.endTransmission();
    */

    MAX30101::InterruptStatus interruptStatus;
    interruptStatus.CheckStatus();

    //if (BIT(data, 7) == 1){ // If FIFO Buffer is almost full, collect data
    //if (BIT(data, 6) == 1){ // If NewSample, collect data
    String outSentence = "";
    uint8_t readPtr;
    uint8_t writePtr;
    uint8_t overflowCtr;
    //uint32_t redLEDBuf;
    //uint32_t irLEDBuf;
    //uint32_t greenLEDBuf;
    MAX30101::FIFOData ledDataBuf;
    uint8_t dataAval;

    /*MAX30101::read_reg(REG_FIFO_RD_PTR, &readPtr);
      MAX30101::read_reg(REG_FIFO_WR_PTR, &writePtr);
      MAX30101::read_reg(REG_OVF_COUNTER, &overflowCtr);*/

    Wire.beginTransmission(0x57);
    Wire.write(REG_FIFO_WR_PTR);
    Wire.endTransmission();
    Wire.requestFrom(0x57, 3);
    writePtr = Wire.read();
    overflowCtr = Wire.read();
    readPtr = Wire.read();
    Wire.endTransmission();

    if (readPtr != writePtr) {
      /*outSentence += "PH,";
      outSentence += millis();
      outSentence += ",";
      outSentence += overflowCtr;
      outSentence += ",";*/
      if (readPtr < writePtr) {
        dataAval = writePtr - readPtr;
      } else {
        dataAval = 32 - readPtr + writePtr;
      }
      /*outSentence += dataAval;
      outSentence += "\r\n";*/
      //Serial.print(outSentence);

      if (dataAval == 1 && startLogging[c] == 0){
        sampleTime[c] = millis();
        startLogging[c] = 1;
      }

      // Send data
      if (startLogging[c] == 1){
        while (dataAval > 0) {
          outSentence += "PL,";
          sampleNo[c]++;          
          outSentence += sampleNo[c];
          outSentence += ",";
          if (sampleNo[c] > 1){
            sampleTime[c] += 10;
          }
          outSentence += sampleTime[c];
          outSentence += ",";
          MAX30101::ReadData(ledDataBuf);
          outSentence += ledDataBuf.slot1;
          outSentence += ",";
          outSentence += ledDataBuf.slot2;
          outSentence += ",";
          outSentence += ledDataBuf.slot3;
          outSentence += ",";
          outSentence += ledDataBuf.slot4;
          outSentence += ",";
          outSentence += overflowCtr;
          outSentence += "\r\n";
  
          /*if(dataAval>1){
            outSentence+=",";
            } else {
            outSentence+="\r\n";
            }*/
          dataAval--;
        }
      }
      Serial.print(outSentence);
    }

    //if (BIT(data2, 1) == 1) {   // If temperature conversion ended
    if (interruptStatus.DieTempReady()) {   // If temperature conversion ended
      /*uint8_t tempInt;
      uint8_t tempFrac;
      String outSentence = "";

      MAX30101::read_reg(REG_TEMP_INTR, &tempInt);
      MAX30101::read_reg(REG_TEMP_FRAC, &tempFrac);

      outSentence += "PT,";
      outSentence += millis();
      outSentence += ",";
      outSentence += tempInt;
      outSentence += ",";
      outSentence += tempFrac;
      outSentence += "\r\n";
      Serial.print(outSentence);

      MAX30101::write_reg(REG_TEMP_CONFIG, 0x01);*/
      float tempFloat = MAX30101::DieTempConvRetrieveFloat();
      int32_t tempInt = MAX30101::DieTempConvRetrieveInt();
      Serial.printf("T,%d,%F,%d\n", millis(), tempFloat, tempInt);
      MAX30101::DieTempConvRequest();
    }
}
