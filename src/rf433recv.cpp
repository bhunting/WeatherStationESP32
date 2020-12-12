
#include <Arduino.h>

// Incoming bit stream read from 433 MHz radio
// ring buffer size has to be large enough to fit
// data and sync signal, at least 120 bytes,
// round up to 128 for now
#define RING_BUFFER_SIZE 152
#define SYNC_HIGH (600) // pulse lengths for OOK from RF radio
#define SYNC_LOW (600)
#define SYNC_MAX (600)
#define PULSE_LONG 400
#define PULSE_SHORT 220
#define PULSE_RANGE 100
#define PULSE_VALID_MAX (SYNC_MAX+PULSE_RANGE)
#define PULSE_VALID_MIN (PULSE_SHORT-PULSE_RANGE)

//#define BIT1_HIGH       PULSE_LONG
//#define BIT1_LOW        PULSE_SHORT
//#define BIT0_HIGH       PULSE_SHORT
//#define BIT0_LOW        PULSE_LONG

//BIT1 = pulse timing pattern to be interpreted as a 1
#define BIT1_HIGH_MIN (PULSE_LONG - PULSE_RANGE)
#define BIT1_HIGH_MAX (PULSE_LONG + PULSE_RANGE)
#define BIT1_LOW_MIN (PULSE_SHORT - PULSE_RANGE)
#define BIT1_LOW_MAX (PULSE_SHORT + PULSE_RANGE)

//BIT0 = pulse timing pattern to be interpreted as a 0
#define BIT0_HIGH_MIN (PULSE_SHORT - PULSE_RANGE)
#define BIT0_HIGH_MAX (PULSE_SHORT + PULSE_RANGE)
#define BIT0_LOW_MIN (PULSE_LONG - PULSE_RANGE)
#define BIT0_LOW_MAX (PULSE_LONG + PULSE_RANGE)

// On the ESP connect the data pin, the pin that will be
// toggling with the incomming data from the RF module, to
// and interrupt capable digital pin.
// Pin D1 on the ESP8266 is interrupt capable and can be configured
// for interrupt on change, change to high or low.
// The squelch pin is an input to the radio that squelches, or
// blanks the incoming data stream. Use the squelch pin to
// stop the data stream and prevent interrupts between the
// data packets if desired.
//
#define DATAPIN (D1)                      // data pint D1 (GPIO5) on D1 mini
#define SQUELCHPIN (D2)                   // Squelch pin D2 (GPIO4) on D1 mini
#define SYNCPULSECNT (4)                  // 4 pulses (8 edges)
#define SYNCPULSEEDGES (SYNCPULSECNT * 2) // Specific format of the Accurite sensors

#define DATABYTESCNT_MIN (7)                   // Minimum number of data bytes
#define DATABITSCNT_MIN (DATABYTESCNT_MIN * 8) // 7 bytes * 8 bits
#define DATABITSEDGES_MIN (DATABITSCNT_MIN * 2)

#define DATABYTESCNT_MID (8)                   // Minimum number of data bytes
#define DATABITSCNT_MID (DATABYTESCNT_MID * 8) // 8 bytes * 8 bits
#define DATABITSEDGES_MID (DATABITSCNT_MID * 2)

#define DATAEDGECNT_MID 128 //8 Bytes x 8 bits per byte x 2 edges per bit

#define DATABYTESCNT_MAX (9)                   // 9 Bytes
#define DATABITSCNT_MAX (DATABYTESCNT_MAX * 8) // 9 bytes * 8 bits
#define DATABITSEDGES_MAX (DATABITSCNT_MAX * 2)

// buffers and indexes for reading bits from RF 433 MHz radio
// The pulse durations are the measured time in micro seconds between
// pulse edges.
static volatile unsigned long pulseDurations[RING_BUFFER_SIZE];
static volatile unsigned int syncIndex = 0; // index of the last bit time of the sync signal
static volatile unsigned int dataIndex = 0; // index of the first bit time of the data bits (syncIndex+1)
static volatile bool syncFound = false;     // true if sync pulses found
static volatile bool receivedBits = false;      // true if sync plus enough bits found
static volatile unsigned int changeCount = 0;
static volatile unsigned int bytesReceived = 0;

/*
 * helper code to print formatted hex 
 */
void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex
{
  char tmp[length * 2 + 1];
  byte first;
  int j = 0;
  for (uint8_t i = 0; i < length; i++)
  {
    first = (data[i] >> 4) | 48;
    if (first > 57)
      tmp[j] = first + (byte)39;
    else
      tmp[j] = first;
    j++;

    first = (data[i] & 0x0F) | 48;
    if (first > 57)
      tmp[j] = first + (byte)39;
    else
      tmp[j] = first;
    j++;
  }
  tmp[length * 2] = 0;
  Serial.print(tmp);
}

/* PART OF ISR (Interrup Handler)
 * Must be defined with ICACHE_RAM_ATTR if called in an ISR.
 * Look for the sync pulse train, 4 high-low pulses of 600 uS high and 600 uS low.
 * idx is index of last captured bit duration.
 * Search backwards 8 times looking for 4 pulses approximately 600 uS long.
 */
ICACHE_RAM_ATTR bool isSync(unsigned int idx)
{
  // check if we've received 4 pulses of matching timing
  for (int i = 0; i < SYNCPULSEEDGES; i += 2)
  {
    unsigned long t1 = pulseDurations[(idx + RING_BUFFER_SIZE - i) % RING_BUFFER_SIZE];
    unsigned long t0 = pulseDurations[(idx + RING_BUFFER_SIZE - i - 1) % RING_BUFFER_SIZE];

    // any of the preceeding 8 pulses are out of bounds, short or long,
    // return false, no sync found
    if (t0 < (SYNC_HIGH - PULSE_RANGE) || t0 > (SYNC_HIGH + PULSE_RANGE) ||
        t1 < (SYNC_LOW - PULSE_RANGE) || t1 > (SYNC_LOW + PULSE_RANGE))
    {
      return false;
    }
  }
  return true;
}

/* Interrupt handler 
 * ESP requires ISR be declared with ICACHE_RAM_ATTR
 * Set to interrupt on edge (level change) high or low transition.
 * Change the state of the LED on each interrupt. 
 * This allows scoping LED pin to see the interrupt / data pulse train.
 */
ICACHE_RAM_ATTR void handler()
{
  static unsigned long duration = 0;
  static unsigned long lastTime = 0;
  static unsigned int ringIndex = 0;
  static unsigned int bitState = 0;

  bitState = digitalRead(DATAPIN);
  digitalWrite(LED_BUILTIN, bitState);

  // ignore if we haven't finished processing the previous
  // received signal in the main loop.
  if (receivedBits == true)
  {
    return;
  }

  // calculating timing since last edge change
  long time = micros();
  duration = time - lastTime;
  lastTime = time;

  // Known error in bit stream is runt/short pulses.
  // If we ever get a really short, or really long,
  // pulse we know there is an error in the bit stream
  // and should start over.
  if ((duration > (PULSE_VALID_MAX)) || (duration < (PULSE_VALID_MIN)))
  {
    receivedBits = false;
    syncFound = false;
    changeCount = 0; // restart looking for data bits
    return; // might as well bail, this is not a good pulse
  }

  // pulse width is within min/max so grab the edge time and
  // store data in ring buffer
  ringIndex = (ringIndex + 1) % RING_BUFFER_SIZE;
  pulseDurations[ringIndex] = duration;
  changeCount++; // found another edge

  // detect sync signal
  if (isSync(ringIndex))
  {
    syncFound = true;
    changeCount = 0; // restart looking for data bits
    syncIndex = ringIndex;
    dataIndex = (syncIndex + 1) % RING_BUFFER_SIZE;
  }

  // If a sync has been found then start looking for the
  // DATABITSEDGES data bit edges.
  if (syncFound)
  {
    // if not enough bits yet, no message received yet
    if (changeCount < DATABITSEDGES_MIN)
    {
      receivedBits = false;
    }
    else if (changeCount > DATABITSEDGES_MIN)
    {
      // if too many bits received then reset and start over
      receivedBits = false;
      syncFound = false;
      changeCount = 0; // restart looking for data bits
    }
    else
    {
      receivedBits = true;
      bytesReceived = 7;
    }
  }
}


/*
 * Convert pulse durations to bits.
 * 1 bit ~0.4 msec high followed by ~0.2 msec low
 * 0 bit ~0.2 msec high followed by ~0.4 msec low
 */
int convertTimingToBit(unsigned int t0, unsigned int t1)
{
  if (t0 > (BIT1_HIGH_MIN) && t0 < (BIT1_HIGH_MAX) && t1 > (BIT1_LOW_MIN) && t1 < (BIT1_LOW_MAX))
  {
    return 1;
  }
  else if (t0 > (BIT0_HIGH_MIN) && t0 < (BIT0_HIGH_MAX) && t1 > (BIT0_LOW_MIN) && t1 < (BIT0_LOW_MAX))
  {
    return 0;
  }
  return -1; // undefined
}


// Print the bit stream for debugging.
// Generates a lot of chatter, normally disable this.
void displayBitTiming()
{
  unsigned int ringIndex;

  Serial.print("syncFound = ");
  Serial.println(syncFound);
  Serial.print("changeCount = ");
  Serial.println(changeCount);
  Serial.print("bytesReceived = ");
  Serial.println(bytesReceived);
  Serial.print("syncIndex = ");
  Serial.println(syncIndex);

  Serial.print("dataIndex = ");
  Serial.println(dataIndex);

  ringIndex = (syncIndex - (SYNCPULSEEDGES - 1)) % RING_BUFFER_SIZE;

  for (unsigned int i = 0; i < (SYNCPULSECNT + (bytesReceived * 8)); i++)
  {
    int bit = convertTimingToBit(pulseDurations[ringIndex % RING_BUFFER_SIZE],
                                 pulseDurations[(ringIndex + 1) % RING_BUFFER_SIZE]);

    Serial.print("bit ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(bit);
    Serial.print(" t1 = ");
    Serial.print(pulseDurations[ringIndex % RING_BUFFER_SIZE]);
    Serial.print(" t2 = ");
    Serial.println(pulseDurations[(ringIndex + 1) % RING_BUFFER_SIZE]);

    ringIndex += 2;
  }
}

void setupRF433()
{
  pinMode(LED_BUILTIN, OUTPUT); // LED output
  pinMode(DATAPIN, INPUT_PULLUP);      // data interrupt input
  pinMode(SQUELCHPIN, OUTPUT);  // data squelch pin on radio module
}

void attachRF433int()
{
  attachInterrupt(digitalPinToInterrupt(DATAPIN), handler, CHANGE);
}

void detachRF433int()
{
    detachInterrupt(digitalPinToInterrupt(DATAPIN));
}

void squelchRF433()
{
  digitalWrite(SQUELCHPIN, LOW); // squelch data
}

void unsquelchRF433()
{
  digitalWrite(SQUELCHPIN, HIGH); // UN-squelch data
}

unsigned int bytesRecvCntRF433()
{
    return bytesReceived;
}

bool receivedBitsRF433()
{
    return receivedBits;
}

bool decodeBitstreamRF433( byte dataBytes[], unsigned int bytesRecvCnt)
{
    unsigned int ringIndex;
    bool bitdecodefail = false; // initialize bit decode error flag to no error

    // clear the data bytes array
    for (unsigned int i = 0; i < bytesRecvCnt; i++)
    {
      dataBytes[i] = 0;
    }

    // syncIndex is index in bit stream of end of sync pulses
    ringIndex = (syncIndex + 1) % RING_BUFFER_SIZE;

    for (unsigned int i = 0; i < bytesRecvCnt * 8; i++)
    {
      int bit = convertTimingToBit(pulseDurations[ringIndex % RING_BUFFER_SIZE],
                                   pulseDurations[(ringIndex + 1) % RING_BUFFER_SIZE]);
      if (bit < 0)
      {
        bitdecodefail = true;
        break; // exit loop
      }
      else
      {
        dataBytes[i / 8] |= bit << (7 - (i % 8));
      }
      ringIndex += 2;
    }

    return bitdecodefail;
}

void resetBitStreamRF433()
{
    receivedBits = false;
    syncFound = false;
    changeCount = 0;
    bytesReceived = 0;
}
