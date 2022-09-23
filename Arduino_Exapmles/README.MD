

## Using **Arduino IDE**

1. Install the current upstream Arduino IDE at the 1.8 level or later. The current version is at the [Arduino website](http://www.arduino.cc/en/main/software).
2. Start Arduino and open Preferences window. In additional board manager add url: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json .separating them with commas.
3. Copy or move the folder in the **libdeps** directory to `<C:\Users\Your User Name\Documents\Arduino\libraries>` 
4. Board select `Tools` -> `Board` -> `ESP32S2 Dev Module`
5. Build and Upload the sketch
6. View the obtained ip address in the serial output,  enter the ip address in the browser to view the data.

