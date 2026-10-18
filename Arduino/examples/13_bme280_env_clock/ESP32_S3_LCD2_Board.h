/**
 * Waveshare ESP32-S3-LCD-2 — board-specific pins and SD helpers.
 *
 * SD SPI lines match Arduino example `03_sd_card_test` (same SCK/MOSI/MISO as LCD, CS=41).
 * Card must be FAT32 (per Waveshare wiki).
 *
 * https://www.waveshare.com/wiki/ESP32-S3-LCD-2
 */

#ifndef ESP32_S3_LCD2_BOARD_H
#define ESP32_S3_LCD2_BOARD_H

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// Shared SPI with LCD (see 02_gfx_helloworld / 03_sd_card_test)
#define ESP32_S3_LCD2_SPI_SCK   39
#define ESP32_S3_LCD2_SPI_MOSI  38
#define ESP32_S3_LCD2_SPI_MISO  40
#define ESP32_S3_LCD2_SD_CS     41
#define ESP32_S3_LCD2_LCD_CS    45

/** Deselect LCD + SD before SD transactions on the shared SPI bus. */
static inline void esp32_s3_lcd2_spi_bus_release(int8_t lcd_cs = ESP32_S3_LCD2_LCD_CS) {
  if (lcd_cs >= 0) {
    pinMode(lcd_cs, OUTPUT);
    digitalWrite(lcd_cs, HIGH);
  }
  pinMode(ESP32_S3_LCD2_SD_CS, OUTPUT);
  digitalWrite(ESP32_S3_LCD2_SD_CS, HIGH);
  delayMicroseconds(4);
}

/**
 * Mount SD over SPI (call after gfx->begin()).
 * @param spi_hz  SPI clock for SD (lower if mount fails or shared bus is noisy)
 */
static inline bool esp32_s3_lcd2_sd_begin(uint32_t spi_hz = 10000000UL, int8_t lcd_cs = ESP32_S3_LCD2_LCD_CS) {
  esp32_s3_lcd2_spi_bus_release(lcd_cs);
  SPI.begin(ESP32_S3_LCD2_SPI_SCK, ESP32_S3_LCD2_SPI_MISO, ESP32_S3_LCD2_SPI_MOSI,
             ESP32_S3_LCD2_SD_CS);
  delay(10);
  if (!SD.begin(ESP32_S3_LCD2_SD_CS, SPI, spi_hz)) {
    return false;
  }
  const uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    SD.end();
    return false;
  }
  return true;
}

/** Call after gfx->begin() on ESP32-S3-LCD-2 (shared SPI; SD-before-LCD can blank the panel). */
static inline bool esp32_s3_lcd2_sd_begin_tries(int8_t lcd_cs = ESP32_S3_LCD2_LCD_CS) {
  const uint32_t freqs[] = {4000000UL, 10000000UL, 20000000UL};
  for (uint32_t f : freqs) {
    SD.end();
    delay(50);
    if (esp32_s3_lcd2_sd_begin(f, lcd_cs)) {
      return true;
    }
    delay(50);
  }
  return false;
}

static inline const char *esp32_s3_lcd2_sd_card_type_name(uint8_t cardType) {
  switch (cardType) {
    case CARD_MMC:
      return "MMC";
    case CARD_SD:
      return "SDSC";
    case CARD_SDHC:
      return "SDHC";
    default:
      return "NONE";
  }
}

#endif
