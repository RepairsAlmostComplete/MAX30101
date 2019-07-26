# **MAX30101 Photoplethymography (PPG) Sensor**
Driver Library for Arduino and Simplelink Microcontrollers

## Table of Contents
[The MAX30101 PPG Sensor](#the-max30101-ppg-sensor)

[Using the MAX30101ACCEVKIT with an External Microcontroller](#using-the-max30101accevkit-with-an-external-microcontroller)

[Common Library Functions](#common-library-functions)

&nbsp;&nbsp;&nbsp;[The Initialiser Object](#the-initialiser-object)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Create an Initialiser Object](#create-an-initialiser-object)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Enable Buffer Full Interrupt](#enable-buffer-full-interrupt)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Enable PPG Ready Interrupt](#enable-ppg-ready-interrupt)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Enable Ambient Light Cancellation Interrupt](#enable-ambient-light-cancellation-interrupt)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Enable Proximity Threshold Triggered Interrupt](#enable-proximity-threshold-triggered-interrupt)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Enable Die Temp Ready Interrupt](#enable-die-temp-ready-interrupt)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Sampling Average](#sampling-average)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Buffer Rollover](#buffer-rollover)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Buffer Almost Full Value](#buffer-almost-full-value)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Mode Control](#mode-control)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[SPO2 ADC Range](#spo2-adc-range)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[SPO2 Sample Rate](#spo2-sample-rate)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[LED Pulse Width](#led-pulse-width)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[LED Pulse Amplitude](#led-pulse-amplitude)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Proximity Mode LED Pulse Amplitude](#proximity-mode-led-pulse-amplitude)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Multi LED Slots](#multi-led-slots)

&nbsp;&nbsp;&nbsp;[Sensor Initialisation](#sensor-initialisation)

&nbsp;&nbsp;&nbsp;[Check Interrupt Status](#check-interrupt-status)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Creating an Interrupt Status Object](#creating-an-interrupt-status-object)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Obtaining the Interrupt Flags](#obtaining-the-interrupt-flags)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Data Buffer Almost Full Flag](#data-buffer-almost-full-flag)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Data Available in Buffer Flag](#data-available-in-buffer-flag)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Ambient Light Overflow Flag](#ambient-light-overflow-flag)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Proximity Threshold Triggered Flag](#proximity-threshold-triggered-flag)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Power Ready Flag](#power-ready-flag)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Die Temp Ready Flag](#die-temp-ready-flag)

&nbsp;&nbsp;&nbsp;[Die Temperature Conversion](#die-temperature-conversion)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Create a Die Temperature Conversion Object](#create-a-die-temperature-conversion-object)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Request Die Temperature Conversion](#request-die-temperature-conversion)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Retrieve the Die Temperature](#retrieve-the-die-temperature)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Retrieve the Die Temperature as Seperate Whole and Fraction](#retrieve-the-die-temperature-as-seperate-whole-and-fraction)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Retrieve the Die Temperature as a Floating Point Integer](#retrieve-the-die-temperature-as-a-floating-point-integer)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Retrieve the Die Temperature as an Integer](#retrieve-the-die-temperature-as-an-integer)

&nbsp;&nbsp;&nbsp;[Data Counters](#data-counters)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Creating a DataCounter Object](#creating-a-datacounter-object)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Requesting the DataCounters](#requesting-the-datacounters)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Retrieving the Write Pointer](#Retrieving-the-write-pointer)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Retrieving the Read Pointer](#Retrieving-the-read-pointer)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Retrieving the Overflow Counter](#Retrieving-the-overflow-counter)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Retrieving the Number of Data Available in the Buffer](#Retrieving-the-number-of-data-available-in-the-buffer)

## The MAX30101 PPG Sensor
The MAX30101 sensor is produced by Maxim Integrated and is designed in biomedical applications for the detection of heart rate and blood oxygen saturation (SpO2).

The sensor consists of three light emitting diodes (LEDs) emitting in the red (660nm), infrared (IR) (880nm) and green (537nm) parts of the electromagnetic spectrum. The sensor contains a single photodiode for detection of light emitted from the LEDs, or ambient light. This PPG sensor along with it's bi-wavelength companion sensor, the MAX30102, are the only PPG sensors with integrated analogue to digital converter (ADC) and I2C interface inside the sensor iteself. These features make the MAX30101 and MAX30102 sensors the smallest fully integrated PPG sensors on the market at about the size of a grain of rice.

[Return to Table of Contents](#table-of-contents)

## Using the MAX30101ACCEVKIT with an External Microcontroller
The development board for the MAX30101 is the MAX30101ACCEVKIT. This development kit contains a small board containing the MAX30101 sensor, included on this board is a 3-axis accelerometer, the LIS2DH. The kit includes a seperate board containing power regulators which step down USB voltage from 5v, to 4.4v, 3.3v and 1.8v required of the sensor board. The 4.4v is wired to the MAX30101 LEDs, the 1.8v is wired to the VDD of the MAX30101, while the 3.3v is wired to the VDD of the accelerometer. Additionally this board contains a microprocessor with custom firmware for intergration with the MAX30101ACCEVKIT software supplied by Maxim Integrated for testing from a Windows computer. Unlike the MAX30102, Maxim Integrated has not supplied demo code for the MAX30101 for Arduino or similar microprocessors, nor is the board included in the MAX30101ACCEVKIT immediately friendly for use with alternative microprocessors.

However, the included daugher board to the sensor board in the MAX30101ACCEVKIT can be made to work with an external microcontroller. In order to make this board work, you need to make a jumper to short the reset pin (pin 1) on J2 to ground (pin 3) on J2 (**Figure 1**). This will hold the onboard microcontroller in reset, allowing an external board to communicate via I2C with the sensors.

**INSERT FIGURE HERE WITH RESET PIN SHORTING**

Additionally, pins need to be added to the daugher board at SDA, SCL, INT1 (if desired), INT2 (if desired) GND and VBUS (5V in, if desired) (**Figure 2**). The SDA and SCL pins need to be connected to the SDA and SCL pins respectively on your microcontroller. Connect GND to the ground on your microcontroller (important, as both must have a common ground). If you want to power the daughter board from your microcontroller, connect the 5V or USB Power pin to the VBUS pin on the daughter board to supply 5V power. If you choose, you can omit the VBUS pin, however you must then power the daugher borad seperatly from your microcontroller, such as through a seperate USB power supply. This is stepped down on the daughter board to the voltages required of the sensor board as mentioned above. If desired the INT1 can be connected to a pin on your microcontroller for physical interrrupt for the MAX30101 sensor, however it is not necessary, nor is it used in this implementation of the driver. Additionally INT2 can be wired to your microcontroller for the acceleromoter physical interrupt. Please see the LIS2DH library for information on INT2.

**INSERT FIGURE HERE WITH SDA, SCL, VBUS, INT1 AND INT2 PIN LOCATIONS**

[Return to Table of Contents](#table-of-contents)

# Common Library Functions

The following are common to both the Arduino and Simplelink libraries

[Return to Table of Contents](#table-of-contents)

## The Initialiser Object
In the MAX30101 library a class named *Initialiser* is used to define the initialisation options for the MAX30101 sensor. The following sections describe how to create an * *Initialiser* * object, and the options available for initialisation of the MAX30101 sensor.

[Return to Table of Contents](#table-of-contents)

### Create an Initialiser Object
An *Initialiser* object is required in order to set the initialisation options of the MAX30101 sensor. This object is then passed into the Initialise function of the library configuring the MAX30101 sensor for use. To create an *Initialiser* object use the following syntax.
```C
MAX30101::Initialiser ObjectName;
```
For example:
```C
MAX30101::Initialiser initOptions;
```

Note: Further examples in this document will assume the *Initialiser* object is named *initOptions*.

[Return to Table of Contents](#table-of-contents)

### Enable Buffer Full Interrupt
This interrupt is used to indicate when the data buffer is almost full. The buffer is almost full when the number of free spaces remaining falls below the number specified in *[Buffer Almost Full Value](#buffer-almost-full-value)*.

To enable the *Enable Buffer Full Interrupt* use the following syntax:

```C
initOptions.IntBuffFull(bool);
```
For example:
```C
initOptions.IntBuffFull(true);
```

[Return to Table of Contents](#table-of-contents)

### Enable PPG Ready Interrupt
This interrut is used to indicate when new data is availabe in the data buffer. To enable the *Enable PPG Ready Interrupt* use the following syntax:

```C
initOptions.IntPPGReady(bool);
```
For example:
```C
initOptions.IntPPGReady(true);
```

[Return to Table of Contents](#table-of-contents)

### Enable Ambient Light Cancellation Interrupt
This interrupt is used to indicate when the photo diode is saturated, indicating that ambient light is affecting the readings. To enable the *Enable Ambient Light Cancellation Interrupt* use the following syntax:

```C
initOptions.IntAmbientLight(bool);
```
For example:
```C
initOptions.IntAmbientLight(true);
```

[Return to Table of Contents](#table-of-contents)

### Enable Proximity Threshold Triggered Interrupt
This interrupt is used to indicate that the proximity threshold has been reached, indicating that something is on the sensor. When enabled, it will only start recording data from the sensor when the proximity threshold has been reached.

To set the *Enable Proximity Threshold Triggered Interrupt*, use the following syntax:

```C
initOptions.IntProximity(bool);
```
For example:
```C
initOptions.IntProximity(true);
```

[Return to Table of Contents](#table-of-contents)

### Enable Die Temp Ready Interrupt
This interrupt enables the ability of the sensor to measure the temperature of the die. This can be used to compensate for temperature variations of the sensor. Once enabled, a request must be made to trigger a conversion of the die temperature into a digital format stored as per the section *[*Request Die Temperature Conversion](#request-die-temperature-conversion)*.

To enable the *Enable Die Temp Ready Interrupt*, use the following syntax:

```C
initOptions.IntDieTempReady(bool);
```
For example:
```C
initOptions.IntDieTempReady(true);
```

[Return to Table of Contents](#table-of-contents) 

### Sampling Average
The *Sampling Average* initialisation option sets the number of samples that are averaged to produce a single sample output from the MAX30101 sensor. Samples that are used to generate the averaged sample are not sent from the MAX30101 sensor, only the averaged value. If you have a sample rate of 100hz, and sampling average set to 8, then you will have an effective sampling rate of 12.5hz as you will only receive 12.5 samples per second from the MAX30101 sensor.

Valid values for *Sampling Average* are 1 (no averaging), 2, 4, 8, 16 and 32.

To set the *Sampling Average* use the following syntax.
```C
initOptions.SampAvg(uint8_t);
```
For example:
```C
initOptions.SampAvg(8);
```

[Return to Table of Contents](#table-of-contents)

### Buffer Rollover
The *Buffer Rollover* option defines if the buffer should rollover when full, and overwrite unread values, or if it should stop generating new data and wait until slots in the buffer have been freed.

Valid values for *Buffer Rollover* are true (rollover on) and false (rollover off);

To set the *Buffer Rollover* use the following syntax.
```C
initOptions.FIFORollover(bool);
```
For example:
```C
initOptions.FIFORollover(true);
```

[Return to Table of Contents](#table-of-contents)

### Buffer Almost Full Value
The *Buffer Almost Full Value* is an interrupt that is flagged when the data buffer is almost full. The number of free samples used to trigger the buffer almost full flag can be set. If the *Buffer Almost Full Flag* is set to 10 samples, then the *Buffer Almost Full Value* interrupt will be triggered when there is 10 free slots left in the data buffer. Since the data buffer is 32 samples in size, the *Buffer Almost Full Value* would be triggered when there is 22 samples in the buffer.

Valid values for *Buffer Almost Full Value* are 0 to 15.

To set the *Buffer Almost Full Value* use the following syntax.
```C
initOptions.FIFOBuffFull(uint8_t);
```
For example:
```C
initOptions.FIFOBuffFull(10);
```

[Return to Table of Contents](#table-of-contents)

### Mode Control
The *Mode Control* option is used to define the mode of operation for the MAX30101 sensor. There is three modes of operation defined below.

Valid values for *Mode Control* are HR, SPO2 and MULTI.

- **HR** - HR mode is used for recording heart rate only from the MAX30101 sensor. This mode uses only the red LED, all other LEDs are disbaled. Using this mode you do not need to set the *Multi LED Slots*.
- **SPO2** - SPO2 mode is used for recording heart rate and blood oxygen saturation (SPO2) from the MAX30101 sensor. This mode uses only the red and IR LEDs, the green LED is disabled.  Using this mode you do not need to set the *Multi LED Slots*.
- **MULTI** - MULTI mode allows custom configuration of the LEDs, enabling use of the green LED alone or along side of the red and IR LEDs. Using this mode you **must** configure the *Multi LED Slots* and *LED Pulse Amplitude* options.

To set the *Mode Control* option, use the following syntax.
```C
initOptions.ModeCtrl(char*);
```
For example:
```C
initOptions.ModeCtrl("MULTI");
```

[Return to Table of Contents](#table-of-contents)

### SPO2 ADC Range
The *SPO2 ADC Range* defined the range of the analogue to digital (ADC) in the MAX30101 sensor. The MAX30101 has a resolution of 18-bits.

Valid valuse for the *SPO2 ADC Range* are 2048, 4096, 8192 and 16384.

| LSB Size (pA) | Full Scale (nA) |
| :---: | :---: |
| 7.81 | 2048 |
| 15.63 | 4096 |
| 31.25 | 8192 |
| 62.5 | 16384 |

To set the *SPO2 ADC Range* use the following syntax.
```C
initOptions.SPO2ADCRange(uint16_t);
```
For example:
```C
initOptions.SPO2ADCRange(8192);
```

[Return to Table of Contents](#table-of-contents)

### SPO2 Sample Rate
The *SPO2 Sample Rate* set the sample rate in Hz. Note that if *Sampling Average* is configured higher than 1, then the effective sample rate at your microcontroller will be lower than this value. See the *[Sampling Average](#sampling-average)* section above for more information.

Also note that when setting the sample rate it is important to take into consideration the number of LEDs in use, the *[LED Pulse Width](#led-pulse-width)*. The *SPO2 Sample Rate* sets the upper bounds of the pulse width time, for example using a single LED at 100Hz, the maximum pulse width time could be 10ms. The higher the sample rate, the shorter the pulse width. At 3200Hz, the maximum pulse width with a single LED would be 312µs, while with 3 LED slots in use the maximum pulse width would be reduced to 104µs. **LINK TO SECTION ON THIS**

Valid values for *SPO2 Sample Rate* are 50, 100, 200, 400, 800, 1000, 1600 and 3200.

To set the *SPO2 Sample Rate* use the following syntax.
```C
initOptions.SPO2SampRate(uint16_t);
```
For example:
```C
initOptions.SPO2SampRate(100);
```

[Return to Table of Contents](#table-of-contents)

### LED Pulse Width
The *LED Pulse Width* sets the lenght of time in µs the LEDs can remain illuminated. Each if the LEDs (red, IR and green) have the same pulse width. The upper limit of the *LED Pulse Width* is linked to the SPO2 Sample Rate and the number of LEDs in use. Additionally, the pulse width indirectly sets the ADC Resolution of each sample as shown in the table below.

Valid values for *LED Pulse Width* are 69µs, 118µs, 215µs and 411µs.

| Pulse Width (Actual) | ADC Resolution |
| :---: | :---: |
| 69µs (68.95µs) | 15 bits |
| 118µs (117.78µs) | 16 bits |
| 215µs (215.44µs) | 17 bits |
| 411µs (410.75µs) | 18 bits |

To set the *LED Pulse Width* use the following syntax.
```C
initOptions.LEDPulseWidth(uint16_t);
```
For example:
```C
initOptions.LEDPulseWidth(118);
```
[Return to Table of Contents](#table-of-contents)

### LED Pulse Amplitude
The *LED Pulse Amplitude* sets the brightness of the LED by adjusting the current supplied to the LED. There are four *LED Pulse Amplitude* slots assigned to each of the three LEDs (red, IR and green) as shown in the table below.

| LED Pulse Amplitude Slot | LED |
| :---: | :---: |
| LED Slot 1 | Red |
| LED Slot 2 | IR |
| LED Slot 3 | Green |
| LED Slot 4 | Green |

Note: Both LED 3 and LED 4 are wired in the MAX30101 to the Green LED, hence LED Slot 3 and 4 adjust the Green amplitude.

Valid values for *LED Pulse Amplitude* are 0.0 to 51.0 in 0.2 increments.

To set the *LED Pulse Amplitude* use the following syntax.
```C
initOptions.LEDAmplitudeRED(uint8_t);
initOptions.LEDAmplitudeIR(uint8_t);
initOptions.LEDAmplitudeGREEN1(uint8_t);
initOptions.LEDAmplitudeGREEN2(uint8_t);
```
For example:
```C
initOptions.LEDAmplitudeRED(7.2);
initOptions.LEDAmplitudeIR(7.2);
initOptions.LEDAmplitudeGREEN1(100);
initOptions.LEDAmplitudeGREEN2(100);
```

[Return to Table of Contents](#table-of-contents)

### Proximity Mode LED Pulse Amplitude
The *Proximity Mode LED Pulse Amplitude* is not defined in the MAX30101 datasheet as the datasheet specifies that the proximity function has been removed. However, the function still appears to be enabled in firmware. Use of this funtion is provided, however it may not be active in your particular sensor board. The *Proximity Mode LED Pulse Amplitude* sets the brighness of the red LED in proximity mode.

Valid values for *Proximity Mode LED Pulse Amplitude* are 0.0 to 51.0 in 0.2 increments.

To set the *Proximity Mode LED Pulse Amplitude* use the following syntax.
```C
initOptions.LEDAmplitudePilot(uint8_t);
```
For example:
```C
initOptions.LEDAmplitudePilot(uint8_t);
```

[Return to Table of Contents](#table-of-contents)

### Multi LED Slots
There are four *Multi LED Slots* in the MAX30101. Each of these slots can be configured as a position where one of the LEDs will be illuminated. During each sample the LED at each position is turned on and a sample is taken by the photodiode and converted by the ADC. Therefore at 100Hz sample rate, each of the four positions is illuminated, sampled by the photodiode and converted by the ADC 100 times. Each slot can be assigned to the red, IR or green LED; or can be disabled. Slots must be assigned in order such that any disabled slots should be after slots assigned to an LED.

Valid values for *Multi LED Slots* are RED, IR, GREEN, DISABLED, NONE.

To set the *Multi LED Slots*, use the following syntax
```C
initOptions.MultiLEDSlot1(char*);
initOptions.MultiLEDSlot2(char*);
initOptions.MultiLEDSlot3(char*);
initOptions.MultiLEDSlot4(char*);
```
For example:
```C
initOptions.MultiLEDSlot1("RED");
initOptions.MultiLEDSlot2("IR");
initOptions.MultiLEDSlot3("GREEN");
initOptions.MultiLEDSlot4("DISABLED");
```

[Return to Table of Contents](#table-of-contents)

## Sensor Initialisation
Once the Initialiser object has been created, and the initialisation options configured, the MAX30101 sensor must be initialised. Initialising the MAX30101 is achieved by calling the *Initialise* method and passing in the Initialise object. The method will return a boolean value as to the success of the initalisation.

To initialse the MAX30101 sensor use the following syntax.
```C
MAX30101::Initialise(MAX30101::Initialiser);
```
For example:
```C
MAX30101::Initialise(initOptions);
```

This can be used with a while loop to wait for the sensor, for example
```C
while (!MAX30101::Initialise(initOptions))
{
    Serial.println("MAX30101 Initialisation Failed, retrying...");
    delay(1000);
}
```

[Return to Table of Contents](#table-of-contents)

## Check Interrupt Status
To check if an interrupt has been flagged we need to check the interrupt status. The interrupt status can be checked by first creating an *InterruptStatus* object and then initiating a *CheckStatus* on the object. The status of each interrupt is then written to the *InterruptStatus* object and can be retrieved from the *InterruptStatus* object.

### Creating an Interrupt Status Object
An *InterruptStatus* object is required in order to check and retrieve the status of the interrupts. To create an *InterruptStatus* object, use the following syntax.
```C
MAX30101::InterruptStatus ObjectName;
```
For example:
```C
MAX30101::Initialiser interrputStatus;
```

Note: Further example in this document will assume the *InterruptStatus* object is named *interruptStatus*.

[Return to Table of Contents](#table-of-contents)

### Obtaining the Interrupt Flags
Use the *CheckStatus* function to obtain and interperate the interrupt flags. This must be called each time you want to check the status of the interrupt. A call to *CheckStatus* will obtain the status of all interrup flags, the status of each flag can then be obtained using the calls to functions detailed below.

To perform a *CheckStatus* use the following syntax:
```C
interruptStatus.CheckStatus();
```

[Return to Table of Contents](#table-of-contents)

### Data Buffer Almost Full Flag
The *FIFOAlmostFull* function returns a boolean value indicating if the data buffer has reached the preset condition for almost full. To obtain the *FIFOAlmostFull* flag status, use the following syntax:
```C
interruptStatus.fifoAlmostFull;
```

[Return to Table of Contents](#table-of-contents)

### Data Available in Buffer Flag
The *FIFODataReady* function returns a boolean value indicating that there is a new sample of data available in the data buffer. To obtain the *FIFODataReady* flag status, use the following syntax:
```C
interruptStatus.fifoDataReady;
```

[Return to Table of Contents](#table-of-contents)

### Ambient Light Overflow Flag
The *AmbientLightOverflow* function returns a boolean value when the photodiode has reached its maximum limit. This indicates that ambient light is affecting the output of the ADC. To obtain the *AmbientLightOverflow* flag status, use the following syntax:
```C
interruptStatus.ambientLightOVF;
```

[Return to Table of Contents](#table-of-contents)

### Proximity Threshold Triggered Flag
The *Proximity* function returns a boolean value when the threshold for proximity is reached, triggering the start of data collection. To obtain the *Proximity* flag status, use the following syntax:
```C
interruptStatus.proximity;
```

[Return to Table of Contents](#table-of-contents)

### Power Ready Flag
The *PowerReady* function returns a boolean value indicting that the sensor is powered up and ready to collect data. To obtain the *PowerReady* flag status, use the following syntax:
```C
interruptStatus.powerReady;
```

[Return to Table of Contents](#table-of-contents)

### Die Temp Ready Flag
The *DieTempReady* function returns a boolean value indicating that a temperature conversion has completed, and the temperature values are ready in the register for collection. A request must be made to initiate a die temperature conversion before one will become available to be read. To request a die temperature conversion see the section *[Die Temperature Conversion](#die-temperature-conversion)*.

To obtain the *DieTempReady* flag status, use the following syntax:
```C
interruptStatus.dieTempReady;
```

[Return to Table of Contents](#table-of-contents)

## Die Temperature Conversion
The sensor contains a built in temperature sensor for reporting on the temperature of the die. This can be used to increase accuracy of data, as temperature can have an effect on accruacty. To obtain a temperature reading you must first *[Request a Die Temperature Conversion](#request-a-die-temperature-conversion)* and then *[Retrieve the Die Temperature](retrieve-the-die-temperature)*.

[Return to Table of Contents](#table-of-contents)

### Create a Die Temperature Conversion Object
Before you can request a die temperature converation you need to create a *DieTempConversion* object to store the temperature. Use the following syntax to create a *DieTempConversion* object:
```C
MAX30101::DieTempConversion Object Name
```
For example:
```C
MAX30101::DieTempConverstion dieTemp;
```

Note: Further examples in this document will assume the *DieTempConverstion* object is named *dieTemp*.

[Return to Table of Contents](#table-of-contents)

### Request Die Temperature Conversion
Before you can *[Retrieve the Die Temperature](retrieve-the-die-temperature)*, you must first request a *Die Temperature Conversion*. Use the following syntax to request a *Die Temperature Conversion*.
```C
dieTemp.Request();
```

[Return to Table of Contents](#table-of-contents)

### Retrieve the Die Temperature
Once you have requested the die temperature, you can then retrieve the die temperature, however before you do, you need to check that the die temperature has been converted by interrogating the *[Die Temp Ready Flag](#die-temp-ready-flag)*. You must retreive the die temperature before you can obtain the values using *GetWhole*, *GetFrac*, *GetFloat* or *GetInt*. Retrieveing the die temperature will store the value in the object so that it can be used in between requests and retrievals of temperature, whereby the retrieved temperature is returned.

To retrieve the die temperature use the following syntax:
```C
dieTemp.Retrieve();
```
For example:
```C
if (interruptStatus.DieTempReady()){
    dieTemp.Retrieve();
}
```

[Return to Table of Contents](#table-of-contents)

#### Retrieve the Die Temperature as Seperate Whole and Fraction
Retrieveing the die temperature as seperate whole and fraction can be suitable when you wish to minimise the size of the variables used to store or transmit the die temperature. In this function, the whole number is returned as an 8 bit signed integer, while the fraction is returned as a seperate 16 bit unsigned integer. To use this function you must pass a int8_t for the whole number and a uint16_t for the fraction into the function.

Use the following syntax to request the die temperature as a seperate whole and fraction:
```C
dieTemp.GetWhole();
dieTemp.GetFrac();
```
For example:
```C
int8_t tempWhole;
uint16_t tempFrac;
if (interruptStatus.DieTempReady()){
    dieTemp.Retrieve();
    tempWhole = dieTemp.getWhole();
    tempFrac = dieTemp.GetFrac();
    dieTemp.Request();
}
```

[Return to Table of Contents](#table-of-contents)

#### Retrieve the Die Temperature as a Floating Point Integer
If you need the die temperature in a human readable format, or as a decimal number for other opreations, the die temperature can be requested in a floating point integer.

Use the following syntax to request the die temperature as a floating point integer.
```C
dieTemp.GetFloat();
```
For example:
```C
float temperature;
if (interruptStatus.DieTempReady()){
    dieTemp.Retrieve();
    temperature = dieTemp.GetFloat();
    dieTemp.Request();
}
```

[Return to Table of Contents](#table-of-contents)

#### Retrieve the Die Temperature as an Integer
If you need the die temperature as a single interger inclusive of the fraction portion, you can request the die temperature as an integer. The integer is returned as a signed 32 bit integer, and shifted by four decimal places. For example a temperature of 32.875°C is represented as 328750.

Use the following syntax to request the die temperature as an integer.
```C
dieTemp.GetInt();
```
For example:
```C
int32_t temperature;
if (interruptStatus.DieTempReady()){
    dieTemp.Retrieve();
    temperature = dieTemp.GetInt();
    dieTemp.Request();
}
```

[Return to Table of Contents](#table-of-contents)

## Data Counters
The data counter object is used to retrieve the *[write pointer](#Retrieving-the-write-pointer)*, the *[read pointer](#Retrieving-the-read-pointer)*, the *[overflow counter](#Retrieving-the-overflow-counter)* and the *[data available](#Retrieving-the-number-of-data-available-in-the-buffer)*.

[Return to Table of Contents](#table-of-contents)

### Creating a DataCounter object
Before you can request the data counters, you must create a *DataCounter* object. To create a *DataCounter* object, use the following syntax.
```C
MAX30101::DataCounter ObjectName;
```
For example:
```C
MAX30101::DataCounter dataCounter
```
Note: Futher examples in this document will assume the *DataCounter* object is named *dataCounter*.

[Return to Table of Contents](#table-of-contents)

### Requesting the DataCounters
Once the *[DataCounter](#creating-a-datacounter-object)* has been created, you need to request the *data counters*, which reads the data from the sensor. To request the *data conuters* use the following syntax.
```C
dataCounter.Request();
```

[Return to Table of Contents](#table-of-contents)

### Retrieving the Write Pointer
The write pointer contains a value indicating the next available position in the buffer where data will be written to. To read the write pointer value use the following syntax.
```C
dataCounter.writePtr;
```
For example:
```C
uint8_t writePointer = dataCounter.writePtr;
```

[Return to Table of Contents](#table-of-contents)

### Retrieving the Read Pointer
The read pointer contains a value indicating the current read postition in the buffer. To read the read pointer value use the following syntax.
```C
dataCounter.readPtr;
```
For example:
```C
uint8_t readPointer = dataCounter.readPtr;
```

[Return to Table of Contents](#table-of-contents)

### Retrieving the Overflow Counter
The overflow counter indicates how many datapoints have been dropped due to the buffer being full. To read the overflow counter value use the following syntax.
```C
dataCounter.overflowCtr;
```
For example:
```C
uint8_t overflowCounter = dataCoutner.overflowCtr;
```

[Return to Table of Contents](#table-of-contents)

### Retrieving the Number of Data Available in the Buffer
The number of data available in the buffer is calculated from the read and write pointers. The struct performs this calculation each time there is a *[request to collect the data counters](#requesting-the-datacounters)*. To read the number of data available in the buffer us the following syntax.
```C
dataCounter.dataAval;
```
For example:
```C
uint8_t dataAvailable = dataCounter.dataAval;
```

[Return to Table of Contents](#table-of-contents)

## Obtaining PPG Data
The data buffer can hold 32 samples, however each of these samples holds the data from a single LED slot. Depending on the *[Mode Control](#mode-control)* selected, the number of slots utilised for one temporal data set will change, and hence the number of temporal data sets that the buffer can hold will change. If in *HR* mode for example, the buffer can store 32 temporal data points, however, if in *MULTI* mode with all four *[Multi LED Slots](#mulit-led-slots)* in use, the buffer can only store 8 temporal data sets. However, the handling of this is take care by the driver which will automatically obtain the right number of samples per request depending on the *[Mode Control](#mode-control)* and *[Multi LED Slots](#mulit-led-slots)* configuration.

Before you can obtain the PPG data, you first need to determine if there is data avilable to read. This can be achieved in two ways, *[checking the interrupt status](#check-interrupt-status]* or *[Retrieving the number of samples available in the buffer](#Retrieving-the-number-of-data-available-in-the-buffer)*. Once we have determined that samples are available in the buffer, we can read the data in the buffer.

[Return to Table of Contents](#table-of-contents)

### The FIFOData Object
The *FIFOData* object is used to read the PPG data and for retrieval of that data. To create an *FIFOData* object use the following syntax.
```C
MAX30101::FIFOData ObjectName;
```
For example:
```C
MAX30101::FIFOData dataBuf;
```
Note: Further examples in this document will assume the FIFO object is named dataBuf;

[Return to Table of Contents](#table-of-contents)

### Read the PPG Data
Before you can obtain the values for the temporal data set, you must first read the data from the buffer. To read data from the buffer use the following syntax.
```C
dataBuf.ReadData();
```

### Read a Single Data Point
Depending on the configuration of *[Mode Control](#mode-control)* and *[Multi LED Slots](#mulit-led-slots)*, there may be one data point per read, or four data points per read. These are seperated into slot1 to slot4. For example if *[Mode Control](#mode-control)* is configured for *HR*, only one slot will be used, and therefore you only need to obtain data from slot1. If *[Mode Control](#mode-control)* is configured for *SpO2*, then two slots will be used, and you will need to obtain data from slot1 (Red) and slot2 (IR). If *[Mode Control](#mode-control)* is configured for *MULTI* mode, then there may be one to four sets of data to collect dependent on how *[Multi LED Slots](#mulit-led-slots)* is configured, in this case you may need to collect data from slot1 to slot4.

To obtain data from the slots, use the following syntax:
```C
dataBuf.slot1;
dataBuf.slot2;
dataBuf.slot3;
dataBuf.slot4;
```