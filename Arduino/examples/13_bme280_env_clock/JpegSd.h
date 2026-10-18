/*******************************************************************************
 * JPEG decode from SD card (JPEGDEC).
 * Library: https://github.com/bitbank2/JPEGDEC
 ******************************************************************************/
#ifndef JPEG_SD_H_
#define JPEG_SD_H_

#include <JPEGDEC.h>
#include <SD.h>

static JPEGDEC _jpegSd;
static File _jpegSdFile;
static int _jpegSdX;
static int _jpegSdY;
static int _jpegSdXBound;
static int _jpegSdYBound;

static void *jpegOpenSdPath(const char *path, int32_t *pFileSize) {
  _jpegSdFile = SD.open(path, FILE_READ);
  if (!_jpegSdFile) {
    return nullptr;
  }
  *pFileSize = _jpegSdFile.size();
  return &_jpegSdFile;
}

static void jpegCloseSdPath(void *pHandle) {
  (void)pHandle;
  if (_jpegSdFile) {
    _jpegSdFile.close();
  }
}

static int32_t jpegReadSdPath(JPEGFILE *pFile, uint8_t *pBuf, int32_t iLen) {
  File *f = static_cast<File *>(pFile->fHandle);
  return (int32_t)f->read(pBuf, iLen);
}

static int32_t jpegSeekSdPath(JPEGFILE *pFile, int32_t iPosition) {
  File *f = static_cast<File *>(pFile->fHandle);
  f->seek(iPosition);
  return iPosition;
}

/** Draw JPEG from full SD path (e.g. "/PICS/photo.jpg"), scaled to fit heightLimit. */
static void jpegDrawSdPath(const char *path, JPEG_DRAW_CALLBACK *jpegDrawCallback, bool useBigEndian,
                           int x, int y, int widthLimit, int heightLimit) {
  _jpegSdX = x;
  _jpegSdY = y;
  _jpegSdXBound = _jpegSdX + widthLimit - 1;
  _jpegSdYBound = _jpegSdY + heightLimit - 1;

  if (!_jpegSd.open(path, jpegOpenSdPath, jpegCloseSdPath, jpegReadSdPath, jpegSeekSdPath,
                    jpegDrawCallback)) {
    return;
  }

  int scale;
  int iMaxMCUs;
  float ratio = (float)_jpegSd.getHeight() / (float)heightLimit;
  if (ratio <= 1.0f) {
    scale = 0;
    iMaxMCUs = widthLimit / 16;
  } else if (ratio <= 2.0f) {
    scale = JPEG_SCALE_HALF;
    iMaxMCUs = widthLimit / 8;
  } else if (ratio <= 4.0f) {
    scale = JPEG_SCALE_QUARTER;
    iMaxMCUs = widthLimit / 4;
  } else {
    scale = JPEG_SCALE_EIGHTH;
    iMaxMCUs = widthLimit / 2;
  }
  if (iMaxMCUs < 1) {
    iMaxMCUs = 1;
  }
  _jpegSd.setMaxOutputSize(iMaxMCUs);
  if (useBigEndian) {
    _jpegSd.setPixelType(RGB565_BIG_ENDIAN);
  }
  _jpegSd.decode(x, y, scale);
  _jpegSd.close();
}

#endif
