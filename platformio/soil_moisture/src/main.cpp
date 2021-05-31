/***************************************************
 Firmware for Soil Moisture Guard
 
 The board has four channels. Each channel has a
 capacitive moisture sensor, a potentiometer to set
 the desired soil moisture level and a water pump.
 If the moisture drops below the desired level the
 pump is triggered for a given amount of time. If
 that does not restore the moisture level on the first attempt
 a configurable number of pump retries are attempted before
 it gives up.
 
 A display shows the status of all 4 channels:
 - moisture level as bar graph
   black bar when ok, red when too dry
 - moisture level as numeric percentage
   black font when ok, red when too dry
 - number of pump attempts
   black font when ok, red when the maximum number of retries is exceeded
 ****************************************************/

#include <Adafruit_ThinkInk.h>
#include <Adafruit_SleepyDog.h>

#define VERSION "v0.1" // TODO: get from git
//#define DEBUG 1 // comment in for debug info on the Serial port

// Configuration
#define NUMBER_OF_CHANNELS 4
#define PUMP0_MAX_ATTEMPTS 3
#define PUMP0_DURATION     10
#define PUMP1_MAX_ATTEMPTS 3
#define PUMP1_DURATION     10
#define PUMP2_MAX_ATTEMPTS 3
#define PUMP2_DURATION     10
#define PUMP3_MAX_ATTEMPTS 3
#define PUMP3_DURATION     10

#define EPD_CS      9
#define EPD_DC      10
#define SRAM_CS     6
#define EPD_RESET   8
#define EPD_BUSY    7

#define DISP_ENA    A6
#define DEC_EN      A5
#define MOIST_REF   A4

// see CLKPR chapter in ATmega328P manual
// register | division factor
// ---------|----------------
//     0x00 |   1 ->  16MHz
//     0x01 |   2 ->   8MHz
//     0x02 |   4 ->   4MHz
//     0x03 |   8 ->   2MHz
//     0x04 |  16 ->   1MHz
//     0x05 |  32 -> 500kHz
//     0x06 |  64 -> 250kHz
//     0x07 | 128 -> 125kHz
//     0x08 | 256 ->  62kHz
const uint8_t clk_div    = 0x03; // Divide 16MHz for power saving ..
const uint16_t clk_scaler = pow(2, clk_div); // ..  but all delays need to be scaled
#define BAR_SCALER 1.38 // 100% moisture level is 138 pixels wide on the display

// Calibration
#define WET_MEASUREMENT 150 // sensor submersed in water -> 100% moisture level
#define DRY_MEASUREMENT 660 // sensor in dry air -> 0% moisture level
#define NUMBER_OF_MEASUREMENT_SAMPLES 4 // sensors are noisy, sample a few times and take average. Keep < 64 (2^16 uint16_t / 2^10 ADC resolution = 2^6)

// 1.54" Monochrome displays with 200x200 pixels and SSD1681 chipset
ThinkInk_154_Tricolor_Z90 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

// Global state
struct ChannelState_T {
  const uint8_t pump_duration;
  const uint8_t max_pump_attempts;
  uint8_t pump_attempts;
  uint16_t moisture_level;
  uint16_t moisture_level_raw;
  uint16_t moisture_reference_level;
  const uint8_t sensor_analog_pin;
  const uint8_t sensor_dec;
  const uint8_t pump_dec;
} channel_state[NUMBER_OF_CHANNELS] = {
  // init values
  {PUMP0_DURATION, PUMP0_MAX_ATTEMPTS, 0, 99, 0, 25, A0, 4, 0}, // Channel 0
  {PUMP1_DURATION, PUMP1_MAX_ATTEMPTS, 0, 99, 0, 25, A1, 5, 1}, // Channel 1
  {PUMP2_DURATION, PUMP2_MAX_ATTEMPTS, 0, 99, 0, 25, A2, 6, 2}, // Channel 2
  {PUMP3_DURATION, PUMP3_MAX_ATTEMPTS, 0, 99, 0, 25, A3, 7, 3}  // Channel 3
};

///////////////////////////////////////////////////////////////////////////////
// code section below
/////////////////////

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial) { delay(10); }
#endif
  display.begin(THINKINK_TRICOLOR);
  // display.setRotation(1); // experiment with this depending on how the board is installed. Values can be 0, 1, 2, 3
  CLKPR = 0x80;
  CLKPR = clk_div;
#ifdef DEBUG
  Serial.print("Soil Moisture Guard ");
  Serial.println(VERSION);
  Serial.print("Clock divisor ");
  Serial.println(clk_scaler);
#endif
  pinMode(DISP_ENA, OUTPUT);
  pinMode(DEC_EN, OUTPUT);
  pinMode(2, OUTPUT); pinMode(3, OUTPUT); pinMode(4, OUTPUT);
}


uint8_t convertMeasurementToPercent(uint16_t measurement) {
  // saturates at 0 or 99%
  const float a = 100.0/(WET_MEASUREMENT - DRY_MEASUREMENT); // coefficient for 100%
  const float b = -DRY_MEASUREMENT*a;
  float percentage = measurement*a + b;
  return (uint8_t)max(0, min(99, percentage));
}


void setDecoder(uint8_t val) {
  digitalWrite(DEC_EN, LOW);
  uint8_t d4_lvl = (val>>2) & 0x01;
  uint8_t d3_lvl = (val>>1) & 0x01;
  uint8_t d2_lvl = val & 0x01;
#ifdef DEBUG
  Serial.print("setDecoder to ");
  Serial.print(val);
  Serial.print(" d4,d3,d2 = ");
  Serial.print(d4_lvl);
  Serial.print(d3_lvl);
  Serial.println(d2_lvl);
#endif
  // HC237 wiring: A0 - d2, A1 - d3, A2 - d4
  digitalWrite(2, d2_lvl); // this can probably be done in one cycle by writing the appropriate register directly. But we're in no hurry and can use the Arduino API for readability.
  digitalWrite(3, d3_lvl);
  digitalWrite(4, d4_lvl);
  digitalWrite(DEC_EN, HIGH);
}


boolean almostEqual(uint8_t a, uint8_t b, uint8_t absdiff) {
    return (max(a,b) - min(a,b)) <= absdiff;
}


boolean updateState() {
  // get all measurements, return true if there was an update
  boolean update = false;
  for (uint8_t i = 0; i < NUMBER_OF_CHANNELS; i++) {
    setDecoder(channel_state[i].sensor_dec); // power up the sensor and potentiometer
#ifdef DEBUG
    Serial.print("Reading Channel");
    Serial.println(i+1);
#endif
    delay(2000/clk_scaler);                  // wait until oscillator on sensor is steady
    uint16_t measurement     = 0;            // for A/D reading from sensor
    uint16_t measurement_ref = 0;            // for A/D reading from potentiometer (=reference)
    for (uint8_t j = 0; j < NUMBER_OF_MEASUREMENT_SAMPLES; j++) {
      measurement     += analogRead(channel_state[i].sensor_analog_pin);
      measurement_ref += analogRead(MOIST_REF);
    }
    digitalWrite(DEC_EN, LOW); // sensor power down
    measurement     /= NUMBER_OF_MEASUREMENT_SAMPLES; // average
    measurement_ref /= NUMBER_OF_MEASUREMENT_SAMPLES; // the reference shouldn't be noisy so it's not really necessary to average it but .. do it anyway for .. reasons
    uint8_t percentage     = convertMeasurementToPercent(measurement);
    uint8_t percentage_ref = measurement_ref/10; // 0 - 1023 / 10 = 0 - 102%
    // almostEqual allows some tolerance so we don't run pumps for single percentage point changes.
    // Those could be noise. Try to be quiet as much as possible.
    if (!almostEqual(channel_state[i].moisture_level, percentage, 2)) {
      channel_state[i].moisture_level = percentage;
      channel_state[i].moisture_level_raw = measurement;
      update |= true;
    }
    if (!almostEqual(channel_state[i].moisture_reference_level, percentage_ref, 2)) {
      channel_state[i].moisture_reference_level = percentage_ref;
      update |= true;
    }
    if (channel_state[i].moisture_level >= channel_state[i].moisture_reference_level) {
      channel_state[i].pump_attempts = 0;
    } else {
        update |= true; // always go through the update cycle when too dry (pump attempt + counting)
    }
#ifdef DEBUG
    Serial.print("Channel ");
    Serial.print(i+1);
    Serial.print(" raw: ");
    Serial.print(measurement);
    Serial.print(" percent: ");
    Serial.print(percentage);
    Serial.print(" reference percent: ");
    Serial.println(percentage_ref);
#endif
  }
  return update;
}


void updateDisplay() {
  digitalWrite(DISP_ENA, HIGH);
  display.powerUp();
  display.clearBuffer();
  // the display is divided into 4 columns: (1) channel number, (2) moisture level graph, (3) moisture level numeric, (4) pump attempts
  const uint8_t channel_number_x_offset =   0;                 // (1)
  const uint8_t moist_lvl_bar_x_offset  =  12;                 // (2)
  const uint8_t moist_lvl_txt_x_offset  = 150;                 // (3)
  const uint8_t pump_attempts_x_offset  = display.width() - 6; // (4)
  for (uint8_t i = 0; i < NUMBER_OF_CHANNELS; i++) {
    uint8_t y_offset = i*(display.height()/NUMBER_OF_CHANNELS);
    display.setTextSize(2);
    // channel numbers
    display.setTextColor(EPD_BLACK);
    display.setCursor(channel_number_x_offset, y_offset+15);
    display.print(i+1);
    // moisture levels
    if (channel_state[i].moisture_level < channel_state[i].moisture_reference_level) {
      display.setTextColor(EPD_RED);
    } else {
      display.setTextColor(EPD_BLACK);
    }
    display.setCursor(moist_lvl_txt_x_offset, y_offset+15);
    display.print(channel_state[i].moisture_level);
    display.print("%");
    // raw value
    display.setTextSize(1);
    display.setTextColor(EPD_BLACK);
    display.setCursor(moist_lvl_txt_x_offset, y_offset+35);
    display.print(channel_state[i].moisture_level_raw);
    // pump attempts
    display.setTextSize(1);
    if (channel_state[i].pump_attempts >= channel_state[i].max_pump_attempts) {
      display.setTextColor(EPD_RED);
    } else {
      display.setTextColor(EPD_BLACK);
    }
    display.setCursor(pump_attempts_x_offset, y_offset+20);
    display.print(channel_state[i].pump_attempts);
    // moisture level as bar chart
    uint16_t barColor = EPD_BLACK;
    if (channel_state[i].moisture_level < channel_state[i].moisture_reference_level) {
      barColor = EPD_RED;
    }
    uint8_t barLength = (uint8_t)(channel_state[i].moisture_level*BAR_SCALER);
    display.fillRect(moist_lvl_bar_x_offset, y_offset+8, barLength, 34, barColor);
    // reference markers
    barLength = (uint8_t)(channel_state[i].moisture_reference_level*BAR_SCALER);
    display.fillTriangle(moist_lvl_bar_x_offset + barLength - 3, y_offset+4,
                         moist_lvl_bar_x_offset + barLength + 3, y_offset+4,
                         moist_lvl_bar_x_offset + barLength    , y_offset+8,
                         EPD_BLACK);
    display.fillTriangle(moist_lvl_bar_x_offset + barLength    , y_offset+43,
                         moist_lvl_bar_x_offset + barLength + 3, y_offset+47,
                         moist_lvl_bar_x_offset + barLength - 3, y_offset+47,
                         EPD_BLACK);
    display.drawLine(moist_lvl_bar_x_offset + barLength, y_offset+8,
                     moist_lvl_bar_x_offset + barLength, y_offset+43,
                     EPD_BLACK);
  }
  // version in lower right corner
  display.setTextColor(EPD_BLACK);
  display.setCursor(display.width()-strlen(VERSION)*6, display.height()-8);
  display.print(VERSION);
  // draw
  display.display();
  display.powerDown();
  digitalWrite(DISP_ENA, LOW);
}


void runPumps() {
  for (uint8_t i = 0; i < NUMBER_OF_CHANNELS; i++) {
    if (channel_state[i].moisture_level < channel_state[i].moisture_reference_level) {
      if (channel_state[i].pump_attempts < channel_state[i].max_pump_attempts) {
        channel_state[i].pump_attempts += 1;
#ifdef DEBUG
        Serial.print("Channel ");
        Serial.print(i+1);
        Serial.print(" running pump for ");
        Serial.print(channel_state[i].pump_duration);
        Serial.println(" sec");
#endif
        setDecoder(channel_state[i].pump_dec); // pump on
        delay(channel_state[i].pump_duration*1000/clk_scaler);
        digitalWrite(DEC_EN, LOW); // pump off
      }
#ifdef DEBUG
      else {
        Serial.print("Channel ");
        Serial.print(i+1);
        Serial.print(" exceeded maximum number of attempts ");
        Serial.println(channel_state[i].max_pump_attempts);
      }
#endif
    }
  }
}

void loop() {
  boolean hasStateUpdate = updateState();
  if (hasStateUpdate) {
    updateDisplay();
    runPumps();
  }
#ifdef DEBUG
  Serial.println("Finished cycle. Going to sleep");
  //Watchdog.sleep(2000);
  delay(1000);
  Serial.println("Woke up. Starting new cycle.");
#else
  // sleep for 10mins. Longest watchdog-sleep is 8s so we loop a few times
  for (uint8_t i = 0; i < 10*60/8; i++) {
    Watchdog.sleep(8000);
  }
#endif  
}
