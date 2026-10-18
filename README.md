# Solar Manager

Sketch I run on the Waveshare ESP32-S3-LCD-2. It sits on the battery and shows pack voltage, a rough idea of charge current, and whatever sensors I have soldered on.

The screen is the little HUD. The fuller view is the web page at http://solarmanager.local/ once it has joined Wi-Fi. GPIO18 cycles the graph on the screen. GPIO15 is the picture button if there are JPEGs in /PICS on the SD card.

Wi-Fi password is not in this repo. Put yours back in the sketch on your machine before you flash. The board still joins "Nomad Box" by default.

## Flash

PlatformIO, from this folder.

USB:

```
pio run -t upload
```

Over Wi-Fi, after the board is awake and already on the network:

```
pio run -e esp32s3_lcd2_ota -t upload
```

## Hardware I actually have

1S pack on the Waveshare battery plug. Voltage is the onboard sense pin, GPIO5, times 3. Charger is a DFRobot solar manager, about 900 mA, fed by a 5 V 1 A panel. There is no current shunt, so the milliamp numbers are a guess from how fast the pack voltage moves. It stops pushing charge near full and just holds the pack there. That part is the charger, not a bug.

I2C is SDA 48, SCL 47.

- BMP280 or BME280 for temp and pressure. A BME also gives humidity, which the air sensor uses.
- BH1750 light sensor, optional. I have not soldered it yet. ADDR to ground is address 0x23.
- CCS811 for eCO2 and TVOC, optional. WAK to ground. It needs a while to warm up. The page says good / ok / poor next to the number.

Logs land on the SD card under /LOG. env.csv is the regular log. charge.csv is when the pack voltage is rising.
