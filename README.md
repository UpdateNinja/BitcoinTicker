| Supported Targets | ESP32 | ESP32-C3 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- |

# Bitcoin Ticker

This project uses wifi connection and connects to coindesk API to download bitcoin price.
After succesfull connection data will be parsed using JSON parser.
Data is printed on SPI TFT display ST7735
Project is using spiffs to use custom fonts.
Project is using NVS to save all time high bitcoin price recorded when devices has been ON.
