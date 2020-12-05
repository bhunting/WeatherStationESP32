#ifndef __DECODER_ACCURITE_H
#define __DECODER_ACCURITE_H

#include <Arduino.h>

/*
 * Validate the CRC value to validate the packet
 */
bool acurite_crc(volatile byte row[], unsigned int recvcnt);

/*
 * Acurite 06045 Lightning sensor Temperature encoding
 * 12 bits of temperature after removing parity and status bits.
 * Message native format appears to be in 1/10 of a degree Fahrenheit
 * Device Specification: -40 to 158 F  / -40 to 70 C
 * Available range given encoding with 12 bits -150.0 F to +259.6 F
 */
int16_t acurite_6045_getTemp(uint8_t highbyte, uint8_t lowbyte);
int16_t acurite_getTemp_6044M(byte hibyte, byte lobyte);
float convCF(float c);
int acurite_6045_strikeCnt(byte strikeByte);
uint8_t acurite_6045_strikeRange(uint8_t strikeRange);
uint16_t acurite_txr_getSensorId(uint8_t hibyte, uint8_t lobyte);
int acurite_5n1_getBatteryLevel(uint8_t byte);
int16_t acurite_getHumidity(uint8_t byte);
float acurite_getWindSpeed_kph(uint8_t highbyte, uint8_t lowbyte);
char const *const getWindDirection_Descr(byte b);
int16_t acurite_getTemp_5n1(byte highbyte, byte lowbyte);
float acurite_getRainfall(uint8_t hibyte, uint8_t lobyte);
String getTimeSpan(unsigned long startMillis, unsigned long endMillis);
float convKphMph(float kph);
void decode_5n1(byte dataBytes[]);
void decode_Acurite_6044(byte dataBytes[]);
void decode_Acurite_6045(byte dataBytes[]);

#endif