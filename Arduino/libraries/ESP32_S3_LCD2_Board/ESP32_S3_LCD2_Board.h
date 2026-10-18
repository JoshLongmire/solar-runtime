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

/**
 * Mount SD over SPI.
 * @param spi_hz  SPI clock for SD (lower if mount fails or shared bus is noisy)
 */
static inline bool esp32_s3_lcd2_sd_begin(uint32_t spi_hz = 10000000UL) {
  pinMode(ESP32_S3_LCD2_SD_CS, OUTPUT);
  digitalWrite(ESP32_S3_LCD2_SD_CS, HIGH);
  SPI.begin(ESP32_S3_LCD2_SPI_SCK, ESP32_S3_LCD2_SPI_MISO, ESP32_S3_LCD2_SPI_MOSI,
             ESP32_S3_LCD2_SD_CS);
  return SD.begin(ESP32_S3_LCD2_SD_CS, SPI, spi_hz);
}

/** Call after gfx->begin() on ESP32-S3-LCD-2 (shared SPI; SD-before-LCD can blank the panel). */
static inline bool esp32_s3_lcd2_sd_begin_tries(void) {
  const uint32_t freqs[] = {4000000UL, 10000000UL, 20000000UL};
  for (uint32_t f : freqs) {
    if (esp32_s3_lcd2_sd_begin(f)) {
      return true;
    }
    SD.end();
    delay(20);
  }
  return false;
}

#endif
