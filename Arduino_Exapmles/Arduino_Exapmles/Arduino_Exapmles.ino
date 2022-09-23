/*
  An example showing rainbow colours on a 1.8" TFT LCD screen
  and to show a basic example of font use.

  Make sure all the display driver and pin connections are correct by
  editing the User_Setup.h file in the TFT_eSPI library folder.

  Note that yield() or delay(0) must be called in long duration for/while
  loops to stop the ESP8266 watchdog triggering.

  #########################################################################
  ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
  #########################################################################
*/

#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include "image.h"
#include <SD.h>
#include <FS.h>
#include "WiFi.h"

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

unsigned long targetTime = 0;
byte red = 31;
byte green = 0;
byte blue = 0;
byte state = 0;
unsigned int colour = red << 11;

#define SDCARD_MISO 13
#define SDCARD_MOSI 11
#define SDCARD_SCLK  12
#define SDCARD_CS   10

// IO14 is connected to the SD card of the board, the power control of the LED is IO pin
#define POWER_PIN  14
#define ADC_PIN     9


SPIClass SDSPI(HSPI);


void setup(void)
{
    Serial.begin(115200);

    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH);

    tft.init();
    tft.fillScreen(TFT_BLACK);
    // tft.pushImage(0, 0, 240, 135,(uint16));
    tft.setRotation(0);
    tft.pushImage(0, 0, 135, 240, (uint16_t *)logo_image);
    // tft.pushColors((uint8_t *)logo_image, sizeof(logo_image));
    tft.setRotation(1);

    tft.setTextDatum(BC_DATUM);
    tft.setCursor(135 / 2, 204 / 2);

    pinMode(SDCARD_MISO, INPUT_PULLUP);
    SDSPI.begin(SDCARD_SCLK, SDCARD_MISO, SDCARD_MOSI, SDCARD_CS);
    if (!SD.begin(SDCARD_CS, SDSPI)) {
        Serial.println("setupSDCard FAIL");
        tft.setTextColor(TFT_RED);
        tft.println("SDCard not found!");
    } else {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.print("setupSDCard PASS . SIZE = ");
        Serial.print(cardSize);
        Serial.println(" MB");
        tft.setTextColor(TFT_GREEN);
        tft.setTextSize(1);

        tft.setTextColor(TFT_WHITE);
        tft.print("SDCard Size:");
        tft.setTextColor(TFT_GREEN);
        tft.print(cardSize);
        tft.setTextColor(TFT_WHITE);
        tft.println(" MB");
    }
    tft.setCursor(135 / 2, tft.getCursorY());
    tft.setTextColor(TFT_WHITE);
    tft.print("PSRAM");
    tft.setTextColor(psramFound() ? TFT_GREEN : TFT_RED);
    tft.println(psramFound() ? " [OK]" : " [FAIL]");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    Serial.println("scan start");
    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(n);
        Serial.println(" networks found");
        for (int i = 0; i < n; ++i) {
            // Print SSID and RSSI for each network found
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
            Serial.print(" (");
            Serial.print(WiFi.RSSI(i));
            Serial.print(")");
            Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
            delay(10);
        }
    }
    Serial.println("");

    delay(3000);
    targetTime = millis() + 1000;

    pinMode(ADC_PIN, ANALOG);

}

void loop()
{


    if (targetTime < millis()) {
        targetTime = millis() + 2000;
        // Colour changing state machine
        for (int i = 0; i < 240; i++) {
            tft.drawFastVLine(i, 0, tft.height(), colour);
            switch (state) {
            case 0:
                green += 2;
                if (green == 64) {
                    green = 63;
                    state = 1;
                }
                break;
            case 1:
                red--;
                if (red == 255) {
                    red = 0;
                    state = 2;
                }
                break;
            case 2:
                blue ++;
                if (blue == 32) {
                    blue = 31;
                    state = 3;
                }
                break;
            case 3:
                green -= 2;
                if (green == 255) {
                    green = 0;
                    state = 4;
                }
                break;
            case 4:
                red ++;
                if (red == 32) {
                    red = 31;
                    state = 5;
                }
                break;
            case 5:
                blue --;
                if (blue == 255) {
                    blue = 0;
                    state = 0;
                }
                break;
            }
            colour = red << 11 | green << 5 | blue;
        }

        // The standard ADAFruit font still works as before
        tft.setTextColor(TFT_BLACK);
        tft.setCursor (12, 5);
        tft.setTextSize(3);
        tft.print("LilyGo ");
        tft.setCursor (85, 34);
        tft.print("ESP32-S2");

        // The new larger fonts do not use the .setCursor call, coords are embedded
        // tft.setTextColor(TFT_BLACK, TFT_BLACK); // Do not plot the background colour

        // Overlay the black text on top of the rainbow plot (the advantage of not drawing the backgorund colour!)
        // tft.drawCentreString("Font size 2", 80, 14, 2); // Draw text centre at position 80, 12 using font 2

        //tft.drawCentreString("Font size 2",81,12,2); // Draw text centre at position 80, 12 using font 2
        tft.setTextSize(2);
        uint16_t val =  analogRead(ADC_PIN);

        // tft.drawFloat()
        tft.setCursor(80, 80);
        tft.print("ADC Raw:");
        tft.println(val);

        // tft.drawCentreString("Font size 4", 80, 30, 4); // Draw text centre at position 80, 24 using font 4

        // tft.drawCentreString("12.34", 80, 54, 6); // Draw text centre at position 80, 24 using font 6

        // tft.drawCentreString("12.34 is in font size 6", 80, 92, 2); // Draw text centre at position 80, 90 using font 2

        // // Note the x position is the top left of the font!

        // // draw a floating point number
        // float pi = 3.14159; // Value to print
        // int precision = 3;  // Number of digits after decimal point
        // int xpos = 50;      // x position
        // int ypos = 110;     // y position
        // int font = 2;       // font number only 2,4,6,7 valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 0 : a p m
        // xpos += tft.drawFloat(pi, precision, xpos, ypos, font); // Draw rounded number and return new xpos delta for next print position
        // tft.drawString(" is pi", xpos, ypos, font); // Continue printing from new x position
    }
}






