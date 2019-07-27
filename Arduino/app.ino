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

uint32_t sampleNo;
uint32_t sampleTime;
uint8_t startLogging;

MAX30101::DieTempConversion dieTemp; // Used to store the die temperature

void setup()
{
    // Setup serial connection
    Serial.begin(921600);

    // Wait for a serial connection to become available
    while(!Serial.available()){

    }

    //Begin I2C
    Wire.begin();
    Wire.setClock(400000);

    Serial.println();
 
    // Configure Initialiser Options
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

    // Setup MAX PPG Sensor
    Serial.print("Initialising PPG Sensor.... ");
    while (!MAX30101::Initialise(initOptions)) {
      Serial.println("Failed, retrying ...");
      delay(1000);
    }

    // Initiate a temperature conversion
    dieTemp.Request();

    Serial.println("Complete");
}

void loop()
{
    // Check the interrupt status of the MAX30101
    MAX30101::InterruptStatus interruptStatus;
    interruptStatus.CheckStatus();

    // Output string, need to change this so we are using a printf instead.
    String outSentence = "";

    // Create FIFOData object
    MAX30101::FIFOData dataBuf;

    // Create DataCounters object and retireve DataCounters
    MAX30101::DataCounters dataCounters;
    dataCounters.Retrieve();
    
    // If there is data to read, then read data
    // (rewrite such that we are checking dataAval > 0)
    if (dataCounters.readPtr != dataCounters.writePtr) {

      if (dataCounters.dataAval == 1 && startLogging == 0){
        sampleTime = millis();
        startLogging = 1;
      }

      // Collect data and send to serial
      // Convert to printf
      if (startLogging == 1){
        while (dataCounters.dataAval > 0) {
          outSentence += "PL,";
          sampleNo++;          
          outSentence += sampleNo;
          outSentence += ",";
          if (sampleNo > 1){
            sampleTime += 10;
          }
          outSentence += sampleTime;
          outSentence += ",";
          dataBuf.ReadData();
          outSentence += dataBuf.slot1;
          outSentence += ",";
          outSentence += dataBuf.slot2;
          outSentence += ",";
          outSentence += dataBuf.slot3;
          outSentence += ",";
          outSentence += dataBuf.slot4;
          outSentence += ",";
          outSentence += dataCounters.overflowCtr;
          outSentence += "\r\n";

          dataCounters.dataAval--;
        }
      }
      Serial.print(outSentence);
    }

    // If Die Temperature Conversion available, collect and ouput to serial
    if (interruptStatus.dieTempReady) {

      dieTemp.Retrieve();
      Serial.printf("T,%d,%F,%d,%d,%d\n", millis(), dieTemp.GetFloat(), dieTemp.GetInt(), dieTemp.GetWhole(), dieTemp.GetFrac());

      dieTemp.Request();
    }
}
