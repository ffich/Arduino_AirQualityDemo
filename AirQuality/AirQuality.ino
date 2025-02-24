//Uncomment only one of the following lines to select your display.
//#define USE_ILI9341   //2.4 inch 320x240 TFT Feather Wing
//#define USE_HX8357    //3.5 inch 480x320 TFT Feather Wing
//#define USE_HALLOWING //Adafruit Hallowing M0 Express
//#define USE_PYGAMER   //Adafruit PyGamer M4 Express
#define USE_GIGA_GFX    //Arduino Giga R1 with display
//#define USE_MCUFRIEND     // 2.8 inch 240x320 MCUFRIEND displays
#include "board_select.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Example program for font collections. Shows mixed font strings,
// and the use of print() and getTextBound() on them.

#include "FontCollection.h"

// Uses libraries:
// Any graphics library inheriting from Adafruit

// Text and UI symbol fonts go into a font collection.
// This one is for 18pt Sans
#include <fonts/FreeSans18pt7b.h>
#include <fonts/UISymbolSans18pt7b.h>
FontCollection fc18(&display, &FreeSans18pt7b, &UISymbolSans18pt7b, 1, 1);

// This one is for 12pt Sans; too small on Giga but useful on the smaller displays
#include <fonts/FreeSans12pt7b.h>
#include <fonts/UISymbolSans12pt7b.h>
FontCollection fc12(&display, &FreeSans12pt7b, &UISymbolSans12pt7b, 1, 1);

// This one is for 24pt Sans
#include <fonts/FreeSans24pt7b.h>
#include <fonts/UISymbolSans24pt7b.h>
FontCollection fc24(&display, &FreeSans24pt7b, &UISymbolSans24pt7b, 1, 1);

// Colours in RGB565.
#define CYAN    (uint16_t)0x07FF
#define RED     (uint16_t)0xf800
#define BLUE    (uint16_t)0x001F
#define GREEN   (uint16_t)0x07E0
#define MAGENTA (uint16_t)0xF81F
#define WHITE   (uint16_t)0xffff
#define BLACK   (uint16_t)0x0000
#define YELLOW  (uint16_t)0xFFE0
#define GREY (uint16_t)display.color565(0x7F, 0x7F, 0x7F)
#define DKGREY (uint16_t)display.color565(0x3F, 0x3F, 0x3F)

// Define color
#define FONT_COLOR YELLOW

// Define Refresh rate
#define REFRESH_RATE_S  10

// Define if use or not splash screen (Arduino logo)
//#define LOGO

// Select font
#define MEDIUM_FONT
//#define BIG_FONT

#ifdef MEDIUM_FONT
  #define X_HOME 20
  #define Y_HOME 50

  #define X_TAB 480
  #define Y_TAB 100
#endif

#ifdef BIG_FONT
  #define X_HOME 20
  #define Y_HOME 20

  #define X_TAB 590
  #define Y_TAB 120
#endif

// Include the NiclaSenseEnv library
#include "NiclaSenseEnv.h"

typedef enum
{
  AIR_QUALITY_VIEW,
  TEMP_HUM_VIEW
} ViewType;

// Global device object for Nicla Sense Env
NiclaSenseEnv device;

// Global variables
int16_t x = 20, y = 20;
int16_t xr, yr;
uint16_t wr, hr;

// Air quality variables
float AirQuality;
float CO2;
float TVOC;

// T&H variables
float Temp;
float Hum;

// View
ViewType View = AIR_QUALITY_VIEW;

// Air quality strings
char AirQuality_s[5];
char CO2_s[5];
char TVOC_s[5];

// T&H strings
char Temp_s[5];
char Hum_s[5];

/**
  Displays air quality data from the ZMOD4410 sensor.
  @param sensor Reference to IndoorAirQualitySensor object controlling the sensor.
*/
void displayAqSensorData(IndoorAirQualitySensor& sensor) {
    if (sensor.enabled()) {
        Serial.print("- Indoor air quality value: ");
        Serial.println(sensor.airQuality());
        Serial.print("- CO2 (ppm): ");
        Serial.println(sensor.CO2());
        Serial.print("- TVOC (mg/m3): ");
        Serial.println(sensor.TVOC());
        Serial.print("- Ethanol (ppm): ");
        Serial.println(sensor.ethanol());
        Serial.println("");
    } else {
        Serial.println("- Indoor air quality sensor is disabled!");
    }
}

/**
  Displays temperature and humidity data from the HS4001 sensor.
  @param sensor Reference to TemperatureHumiditySensor object controlling the sensor.
*/
void displayThSensorData(TemperatureHumiditySensor& sensor) {
    if (sensor.enabled()) {
        float temperature = sensor.temperature();
        if (isnan(temperature)) {
            Serial.println("- Temperature: N/A");
        } else {
            Serial.print("- Temperature: ");
            Serial.print(temperature, 2);
            Serial.println(" °C");
        }
        Serial.print("- Relative humidity: ");
        Serial.print(sensor.humidity(), 2);
        Serial.println(" %");
        Serial.println("");
    } else {
        Serial.println("- Temperature sensor is disabled!");
    }
}

#ifdef LOGO
#include "Arduino_H7_Video.h"
#include "ArduinoGraphics.h"

Arduino_H7_Video Display(800, 480, GigaDisplayShield);
#endif

void setup()
{
#ifdef LOGO


  delay(5000);
#endif

  // Initialize serial communication and wait up to 2.5 seconds for a connection
  Serial.begin(115200);
  for (auto startNow = millis() + 2500; !Serial && millis() < startNow; delay(500));

  if (device.begin()) {
      Serial.println("- Device is connected!");
      auto airQualitySensor = device.indoorAirQualitySensor();

      // Set the sensor mode to indoor air quality
      airQualitySensor.setMode(IndoorAirQualitySensorMode::indoorAirQuality);

      // The ZMOD4410 can take a sample every 3 seconds in IAQ mode and requires 60 warm-up samples,
      // meaning the sensor will take about 3 minutes to fully warm-up before accurate readings can 
      // be obtained. In this example, we allow 5 seconds for the sensor to start delivering data.
      delay(5000);
  } else {
      Serial.println("- Device could not be found. Please double-check the wiring!");
  }

  // Init display
  Initialize_Display();
}

void loop() 
{
  /* ****************************** AQ MODE ****************************** */
  if (View == AIR_QUALITY_VIEW)
  { 
    View = TEMP_HUM_VIEW;  
    // Read data from the ZMOD4410 sensor
    auto airQualitySensor = device.indoorAirQualitySensor();
    displayAqSensorData(airQualitySensor);

    // Fill variables
    if (airQualitySensor.enabled())
    {
      AirQuality = airQualitySensor.airQuality();
      CO2 = airQualitySensor.CO2();
      TVOC = airQualitySensor.TVOC();
    }
    else
    {
      AirQuality = 0;
      CO2 = 0;
      TVOC = 0;
    }

    // Convert data
    sprintf(AirQuality_s, "%.2f", AirQuality);
    sprintf(CO2_s, "%.0f", CO2);
    sprintf(TVOC_s, "%.2f", TVOC); 

    /************************** Plot data **************************/

    display.fillScreen(0);

    // Reset coordinates
    x = X_HOME;
    y = Y_HOME;

#ifdef MEDIUM_FONT
    fc24.drawText("AIR QUALITY DATA (ZMOD4410):", x, y, FONT_COLOR);
    y += Y_TAB/2;
    fc24.drawText("----------------------------------------------", x, y, FONT_COLOR);
    y += Y_TAB;
    fc24.drawText("Air quality index:", x, y, FONT_COLOR);
    fc24.drawText(AirQuality_s, (x + X_TAB), y, FONT_COLOR);

    y += Y_TAB;
    fc24.drawText("CO2 [ppm]:", x, y, FONT_COLOR);
    fc24.drawText(CO2_s, (x + X_TAB), y, FONT_COLOR);

    y += Y_TAB;
    fc24.drawText("TVOC [mg/m3]:", x, y, FONT_COLOR);
    fc24.drawText(TVOC_s, (x + X_TAB), y, FONT_COLOR);
#endif 

#ifdef BIG_FONT
    y += Y_TAB;
    fc24.drawText("Air quality:", x, y, FONT_COLOR, 2);
    fc24.drawText(AirQuality_s, (x + X_TAB), y, FONT_COLOR, 2);

    y += Y_TAB;
    fc24.drawText("CO2(ppm):", x, y, FONT_COLOR, 2);
    fc24.drawText(CO2_s, (x + X_TAB), y, FONT_COLOR, 2);

    y += Y_TAB;
    fc24.drawText("Tvoc(mg/m3):", x, y, FONT_COLOR, 2);
    fc24.drawText(TVOC_s, (x + X_TAB), y, FONT_COLOR, 2);
#endif  

  delay(REFRESH_RATE_S*1000);    
  }

  /* ****************************** T&H MODE ****************************** */
  else if (View == TEMP_HUM_VIEW)
  {
    View = AIR_QUALITY_VIEW;
    // Read data from the HS4001 sensor
    auto ThSensor = device.temperatureHumiditySensor();
    displayThSensorData(ThSensor);  

        // Fill variables
    if (ThSensor.enabled())
    {
      Temp = ThSensor.temperature();
      Hum = ThSensor.humidity();

    }
    else
    {
      Temp = 0;
      Hum = 0;
    }

    // Convert data
    sprintf(Temp_s, "%.0f", Temp);
    sprintf(Hum_s, "%.2f", Hum);  

    /************************** Plot data **************************/

    display.fillScreen(0);

    // Reset coordinates
    x = X_HOME;
    y = Y_HOME;     
 
#ifdef MEDIUM_FONT
    fc24.drawText("TEMP & HUM DATA (HS4001):", x, y, FONT_COLOR);
    y += Y_TAB/2;
    fc24.drawText("-----------------------------------------", x, y, FONT_COLOR);
    y += Y_TAB;
    fc24.drawText("Temperature [C]:", x, y, FONT_COLOR);
    fc24.drawText(Temp_s, (x + X_TAB), y, FONT_COLOR);

    y += Y_TAB;
    fc24.drawText("Humidity [%]:", x, y, FONT_COLOR);
    fc24.drawText(Hum_s, (x + X_TAB), y, FONT_COLOR);
#endif 

#ifdef BIG_FONT
    y += Y_TAB;
    fc24.drawText("Temp(C):", x, y, FONT_COLOR);
    fc24.drawText(Temp_s, (x + X_TAB), y, FONT_COLOR);

    y += Y_TAB;
    fc24.drawText("Hum(%):", x, y, FONT_COLOR);
    fc24.drawText(Hum_s, (x + X_TAB), y, FONT_COLOR);
#endif

  delay(REFRESH_RATE_S*1000);  
  }
}