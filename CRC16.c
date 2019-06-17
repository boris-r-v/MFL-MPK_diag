#include "CRC16.h"

uint16_t CRC16(uint8_t *data, size_t len) 
{ 
  uint16_t crc = 0x0000; 
  size_t j; 
  int i; 
  for (j=len; j>0; j--) { 
    crc ^= (uint16_t)(*data++) << 8; 
    for (i=0; i<8; i++) { 
      if (crc & 0x8000) crc = (crc<<1) ^ 0x8005; 
      else crc <<= 1; 
    } 
  } 
  return (crc); 
} 
