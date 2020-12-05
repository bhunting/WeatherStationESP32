#ifndef __RF433RECV_H
#define __RF433RECV_H

#include <Arduino.h>

#define MAX_DATA_BYTES_RECV (10)
/*
 * helper code to print formatted hex 
 */
// prints 8-bit data in hex
void PrintHex8(uint8_t *data, uint8_t length);

/* PART OF ISR (Interrup Handler)
 * Must be defined with ICACHE_RAM_ATTR if called in an ISR.
 * Look for the sync pulse train, 4 high-low pulses of 600 uS high and 600 uS low.
 * idx is index of last captured bit duration.
 * Search backwards 8 times looking for 4 pulses approximately 600 uS long.
 */
ICACHE_RAM_ATTR bool isSync(unsigned int idx);

/* Interrupt handler 
 * ESP requires ISR be declared with ICACHE_RAM_ATTR
 * Set to interrupt on edge (level change) high or low transition.
 * Change the state of the LED on each interrupt. 
 * This allows scoping LED pin to see the interrupt / data pulse train.
 */
ICACHE_RAM_ATTR void handler();

/*
 * Convert pulse durations to bits.
 * 1 bit ~0.4 msec high followed by ~0.2 msec low
 * 0 bit ~0.2 msec high followed by ~0.4 msec low
 */
int convertTimingToBit(unsigned int t0, unsigned int t1);

// Print the bit stream for debugging.
// Generates a lot of chatter, normally disable this.
void displayBitTiming();

void setupRF433();
void attachRF433int();
void detachRF433int();
void squelchRF433();
void unsquelchRF433();
unsigned int bytesRecvCntRF433();
bool receivedBitsRF433();
bool decodeBitstreamRF433( byte dataBytes[], unsigned int bytesRecvCnt);
void resetBitStreamRF433();

#endif