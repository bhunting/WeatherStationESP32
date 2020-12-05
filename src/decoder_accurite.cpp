
#include "decoder_accurite.h"

// Arduino timing definitions
#define eventTimeoutms 7200000 //Timeout in miliseconds before an event is over

/*
 * Validate the CRC value to validate the packet
 */
bool acurite_crc(volatile byte row[], unsigned int recvcnt)
{
  // sum of first n-1 bytes modulo 256 should equal nth byte
  int sum = 0;

  for (unsigned int i = 0; i < recvcnt - 1; i++)
  {
    sum += row[i];
  }
  if (sum != 0 && sum % 256 == row[recvcnt - 1])
  {
    return true;
  }
  else
  {
    Serial.print("CRC ERROR: Expected: ");
    Serial.print(row[recvcnt - 1], HEX);
    Serial.print(" Got: ");
    Serial.println(sum % 256, HEX);
    return false;
  }
}

//const unsigned int tempOffset = 2.4;    // offset in degrees C
//const unsigned int tempOffset = 0;      // offset in degrees C
//const unsigned int tempOffset10th = 24; // offset in 10th degrees C
const unsigned int tempOffset10th = 0; // offset in 10th degrees C

/*
 * Acurite 06045 Lightning sensor Temperature encoding
 * 12 bits of temperature after removing parity and status bits.
 * Message native format appears to be in 1/10 of a degree Fahrenheit
 * Device Specification: -40 to 158 F  / -40 to 70 C
 * Available range given encoding with 12 bits -150.0 F to +259.6 F
 */
int16_t acurite_6045_getTemp(uint8_t highbyte, uint8_t lowbyte)
{
  int16_t rawtemp = ((highbyte & 0x1F) << 7) | (lowbyte & 0x7F);
  int16_t temp = rawtemp - 1500; // temp in 0.1 degrees
  return temp;
}

int16_t acurite_getTemp_6044M(byte hibyte, byte lobyte)
{
  // range -40 to 158 F
  int16_t highbits = (hibyte & 0x0F) << 7;
  int16_t lowbits = lobyte & 0x7F;
  int16_t rawtemp = highbits | lowbits;
  int16_t temp = rawtemp - 1000; // temp in 0.1 degrees
  return temp;
}

float convCF(float c)
{
  return ((c * 1.8) + 32);
}


// local static variables used by the lightning strike counter
// lightning strike data
static int strikeTot = 0;
static int strikeWrapOffset = 0;
//static int lastStrikeCount = 0;
static bool activeStrikes = false;
static unsigned long strikeLast = 0;

int acurite_6045_strikeCnt(byte strikeByte)
{
  //int strikeTot = 0;
  //int strikeWrapOffset = 0;
  int strikeCnt = strikeByte & 0x7f;
  if (strikeTot == 0)
  {
    //Initialize Strike Counter
    strikeTot = strikeCnt;
    strikeWrapOffset = 0;
    strikeLast = millis();
    activeStrikes = false;
    return 0;
  }
  else if (strikeCnt < strikeTot && strikeCnt > 0)
  {
    /*Strikes wrapped around  
     *  Setting strikeTot to 1 as zero would cause a reset, 
     *   Need to make sure strikeCnt isn't 0 so we don't get 
     *   127 false strikes added to our wrap around
     */
    strikeWrapOffset = (127 - strikeTot) + strikeWrapOffset;
    strikeTot = 1;
    strikeLast = millis();
    activeStrikes = true;
  }
  else if (strikeCnt == strikeTot)
  {
    if (millis() - strikeLast > eventTimeoutms)
    {
      //Reset the Lightning event time its been more than the eventTiemoutms
      strikeTot = strikeCnt;
      strikeWrapOffset = 0;
      activeStrikes = false;
      //strikeLast = millis();
    }
  }
  else
  {
    //strike occured increase lastStrike
    strikeLast = millis();
    activeStrikes = true;
  }
  return (strikeCnt - strikeTot) + strikeWrapOffset;
}

uint8_t acurite_6045_strikeRange(uint8_t strikeRange)
{
  return strikeRange & 0x1f;
}

uint16_t acurite_txr_getSensorId(uint8_t hibyte, uint8_t lobyte)
{
  return ((hibyte & 0x3f) << 8) | lobyte;
}

int acurite_5n1_getBatteryLevel(uint8_t byte)
{
  return (byte & 0x40) >> 6;
}

int16_t acurite_getHumidity(uint8_t byte)
{
  // range: 1 to 99 %RH
  int16_t humidity = byte & 0x7F;
  return humidity;
}

float acurite_getWindSpeed_kph(uint8_t highbyte, uint8_t lowbyte)
{
  // range: 0 to 159 kph
  // raw number is cup rotations per 4 seconds
  // http://www.wxforum.net/index.php?topic=27244.0 (found from weewx driver)
  int highbits = (highbyte & 0x1F) << 3;
  int lowbits = (lowbyte & 0x70) >> 4;
  int rawspeed = highbits | lowbits;
  float speed_kph = 0;
  if (rawspeed > 0)
  {
    speed_kph = rawspeed * 0.8278 + 1.0;
  }
  return speed_kph;
}


// Accurite 5n1 Tower Message Types
#define MT_WS_WD_RF 49 // wind speed, wind direction, rainfall
#define MT_WS_T_RH 56  // wind speed, temp, RH

const float winddirections[] = {315.0, 247.5, 292.5, 270.0,
                                337.5, 225.0, 0.0, 202.5,
                                67.5, 135.0, 90.0, 112.5,
                                45.0, 157.5, 22.5, 180.0};

char const *const acurite_5n1_winddirection_str[] =
    {
        "NW",  // 0  315
        "WSW", // 1  247.5
        "WNW", // 2  292.5
        "W",   // 3  270
        "NNW", // 4  337.5
        "SW",  // 5  225
        "N",   // 6  0
        "SSW", // 7  202.5
        "ENE", // 8  67.5
        "SE",  // 9  135
        "E",   // 10 90
        "ESE", // 11 112.5
        "NE",  // 12 45
        "SSE", // 13 157.5
        "NNE", // 14 22.5
        "S"    // 15 180
};

char const *const getWindDirection_Descr(byte b)
{
  // 16 compass points, ccw from (NNW) to 15 (N),
  // { "NW", "WSW", "WNW", "W", "NNW", "SW", "N", "SSW",
  //   "ENE", "SE", "E", "ESE", "NE", "SSE", "NNE", "S" };
  int direction = b & 0x0F;
  return acurite_5n1_winddirection_str[direction];
}

// local variables used by the rain counter
static int acurite_5n1_raincounter = 0;
static int rainWrapOffset = 0;
static int lastRainCount = 0;
static bool activeRain = false;
static unsigned long rainLast = 0;

int16_t acurite_getTemp_5n1(byte highbyte, byte lowbyte)
{
  // range -40 to 158 F
  int highbits = (highbyte & 0x0F) << 7;
  int lowbits = lowbyte & 0x7F;
  int rawtemp = highbits | lowbits;
  int16_t temp_F = rawtemp - 400;  // temp in 0.1 degrees
  return temp_F;
}

float acurite_getRainfall(uint8_t hibyte, uint8_t lobyte)
{
  // range: 0 to 99.99 in, 0.01 in incr., rolling counter?
  int raincounter = ((hibyte & 0x7f) << 7) | (lobyte & 0x7F);
  if (acurite_5n1_raincounter > 0)
  {
    if (raincounter < acurite_5n1_raincounter)
    {
      rainWrapOffset = lastRainCount - acurite_5n1_raincounter;
      acurite_5n1_raincounter = 1;
      rainLast = millis();
      activeRain = true;
    }
    else if (acurite_5n1_raincounter == raincounter)
    {
      if ((millis() - rainLast) >= eventTimeoutms)
      {
        lastRainCount = raincounter;
        rainWrapOffset = 0;
        activeRain = false;
      }
    }
    else
    {
      rainLast = millis();
      activeRain = false;
    }
    return (raincounter - acurite_5n1_raincounter + rainWrapOffset) * .01;
  }
  else
  {
    acurite_5n1_raincounter = raincounter;
    lastRainCount = raincounter;
    rainLast = millis();
    activeRain = false;
    return 0.0;
  }
}


// macros from DateTime.h
/* Useful Constants */
#define SECS_PER_MIN (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY (SECS_PER_HOUR * 24L)

/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) ((_time_ % SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) (_time_ / SECS_PER_DAY)

String getTimeSpan(unsigned long startMillis, unsigned long endMillis)
{
  String outString;
  long span = (endMillis - startMillis) / 1000;
  outString.concat(numberOfHours(span));
  outString.concat(":");
  outString.concat(numberOfMinutes(span));
  outString.concat(":");
  outString.concat(numberOfSeconds(span));
  return outString;
}

float convKphMph(float kph)
{
  return kph * 0.62137;
}

void decode_5n1(byte dataBytes[])
{
  Serial.print("Acurite 5n1 Tower - ");
  Serial.print(acurite_txr_getSensorId(dataBytes[0], dataBytes[1]), HEX);
  Serial.print("; Windspeed - ");
  Serial.print(convKphMph(acurite_getWindSpeed_kph(dataBytes[3], dataBytes[4])));
  if ((dataBytes[2] & 0x3F) == MT_WS_WD_RF)
  {
    // Wind Speed, Direction and Rainfall
    Serial.print("; Direction - ");
    Serial.print(getWindDirection_Descr(dataBytes[4]));
    Serial.print(" Rainfall - ");
    Serial.print(acurite_getRainfall(dataBytes[5], dataBytes[6]));
    if (activeRain)
    {
      Serial.print("; Last rain event - ");
      Serial.print(getTimeSpan(rainLast, millis()));
    }
    Serial.print(";");
  }
  else
  {
    // Wind speed, Temp, Humidity
    Serial.print("; Temp - ");
    Serial.print(acurite_getTemp_5n1(dataBytes[4], dataBytes[5]));
    Serial.print("; Humidity - ");
    Serial.print(acurite_getHumidity(dataBytes[6]));
    Serial.print("%");
  }

  if ((dataBytes[4] & 0x20) == 0x20)
  {
    Serial.print(" Battery Low;");
  }
  Serial.println();
}

void decode_Acurite_6044(byte dataBytes[])
{
  Serial.print("Acurite 6044 Tower - ");
  Serial.print(acurite_txr_getSensorId(dataBytes[0], dataBytes[1]), HEX);
  Serial.print("; Temp - ");
  Serial.print(acurite_getTemp_6044M(dataBytes[4], dataBytes[5]));
  Serial.print("; Temp - ");
  Serial.print(acurite_getTemp_6044M(dataBytes[4], dataBytes[5])/10.0);
  Serial.print("; Temp - ");
  Serial.print(convCF(acurite_getTemp_6044M(dataBytes[4], dataBytes[5])/10.0));
  Serial.print("; Humidity - ");
  Serial.print(acurite_getHumidity(dataBytes[3]));
  Serial.print(" %;");
  if ((dataBytes[4] & 0x20) == 0x20)
  {
    Serial.print("Battery Low;");
  }
  Serial.println();
}

void decode_Acurite_6045(byte dataBytes[])
{
  Serial.print("Acurite 6045 Lightning - ");
  Serial.print(acurite_txr_getSensorId(dataBytes[0], dataBytes[1]), HEX);
  Serial.print("; Temp - ");
  Serial.print(acurite_6045_getTemp(dataBytes[4], dataBytes[5]));
  Serial.print("; Humidity - ");
  Serial.print(acurite_getHumidity(dataBytes[3]));
  Serial.print(" %; Lightning - ");
  if (((dataBytes[7] & 0x40) == 0x40) || activeStrikes)
  {
    if ((dataBytes[7] & 0x20) == 0x20)
    {
      Serial.print("Interference");
    }
    else
    {
      Serial.print("Detected; Dist - ");
      Serial.print(acurite_6045_strikeRange(dataBytes[7]));
      Serial.print(" miles; Count - ");
      Serial.print(acurite_6045_strikeCnt(dataBytes[6]));
      Serial.print("; Last Strike -  ");
      Serial.print(getTimeSpan(strikeLast, millis()));
      Serial.print(";");
    }
  }
  else
  {
    Serial.print("None; ");
  }
  if ((dataBytes[4] & 0x20) == 0x20)
  {
    Serial.print(" Battery Low;");
  }
  Serial.println();
}
