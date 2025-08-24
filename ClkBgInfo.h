#pragma once
#include <stdint.h>

struct ClkBgInfo {
  bool     valid    = false;
  bool     topDown  = false;
  int32_t  bmpW     = 0, bmpH = 0;
  int32_t  drawW    = 0, drawH = 0;
  uint32_t dataOffset = 0;
  uint32_t rowSize    = 0;   // padded row bytes
  char     path[48]   = {0};
};

extern ClkBgInfo g_clkBgInfo;
