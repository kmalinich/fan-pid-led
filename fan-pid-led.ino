#include "Arduino.h"
#include <SoftwareSerial.h>
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library


// Amount of time (in ms) to wait between updates
#define WAIT 1000

// Amount of time (in ms) to wait between sensor measurements
#define SAMPLE_WAIT 50

// Number of samples to take when measuring temp sensor
#define SAMPLE_COUNT 15

// Enable/disable debug mode
#define DBG false

// Enable/disable LCD base configuration (EEPROM stored values)
#define CONFIG_LCD false

// Debug macro to print messages to serial
#define DEBUG(x)   if (DBG && Serial) { Serial.print(x);   }
#define DEBUGLN(x) if (DBG && Serial) { Serial.println(x); }

// Pins
#define PWM_OUT 3  // PWM output pin
#define RPM_IN  2  // RPM input pin
#define TEMP_IN A0 // TMP36 sensor input pin

// Minimum and maximum fan duty cycle (1..255)
#define DUTY_MIN   5
#define DUTY_MAX 255

// PID control tuning parameters
// Great writeup: https://github.com/CapnBry/HeaterMeter/wiki/PID-Controller-Theory
#define KP 1.0 // Determines how aggressively the PID reacts to current amount of error (Proportional)
#define KI 1.0 // Determines how aggressively the PID reacts to error over time         (Integral)
#define KD 0.5 // Determines how aggressively the PID reacts to change in error         (Derivative)


// Temperature target for PID control
double temp_target = 45;

// Temperature minimum and maximum for RGB control
unsigned int temp_min = 20;
unsigned int temp_max = 63;

// Init measurement sample array
int samples[SAMPLE_COUNT];

// Computed sensor temperature
double temp_c;
double temp_f;

// Fan RPM
unsigned int fan_speed;

// Fan hall effect sensor pulse duration
unsigned long pulse_duration;

// PWM duty value
double duty;

// Time placeholder
signed long time_prev;



// Initialize PID library
PID fanPID(&temp_c, &duty, &temp_target, KP, KI, KD, REVERSE);

// Create a software serial port for the LCD
SoftwareSerial lcd = SoftwareSerial(0, 11);


void setup_lcd() {
  // Open LCD SoftwareSerial port
  lcd.begin(9600);

  if (CONFIG_LCD) {
  // Set display size (16x2)
  // lcd.write(0xFE); lcd.write(0xD1);
  // lcd.write(16); lcd.write(2);
  // delay(10);

  // Set and save contrast (0..255)
  // lcd.write(0xFE); lcd.write(0x91);
  // lcd.write(205);
  // delay(10);

  // Set and save brightness (0..255)
  // lcd.write(0xFE); lcd.write(0x99);
  // lcd.write(255);
  // delay(10);

  // Clear startup screen
  // lcd.write(0xFE); lcd.write(0x40);
  // lcd.write("                                ");
  // delay(10);
  }

  
  // Disable cursors
  lcd.write(0xFE); lcd.write(0x4B);
  lcd.write(0xFE); lcd.write(0x54);
  delay(10);

  // Disable auto-scroll
  lcd.write(0xFE); lcd.write(0x52);
  delay(10);

  // Clear screen
  lcd.write(0xFE);
  lcd.write(0x58);

  delay(100);

  // Set initial backlight
  lcd_rgb(0xFF, 0x00, 0xFF);

  // Send init text
  lcd_text("  INITIALIZING  ", "   LEDFan 3.0   ");
}

void lcd_text(String string1, String string2) {
  // Go to 'home' position
  lcd.write(0xFE);
  lcd.write(0x48);
  delay(10);

  lcd.print(string1); delay(10);
  lcd.print(string2);
}

// Calculate RGB values from temperature
void lcd_rgb_calc() {
  // Set full blue and return here if under minimum temp
  if (temp_c < temp_min) {
    lcd_rgb(0, 0, 255);
    return;
  }

  // Set full red and return here if over maximum temp
  if (temp_c > temp_max) {
    lcd_rgb(255, 0, 0);
    return;
  }

  // Set full green and return here if at target temp
  if (round(temp_c) == temp_target) {
    lcd_rgb(0, 255, 0);
    return;
  }

  // Calculate "rgb coefficient"
  float temp_calc = (temp_c - temp_min) / (temp_max - temp_min);

  // Under temp target
  if (temp_calc <= 0.5) {
    lcd_rgb(0, (510 * temp_calc), (255 - (510 * temp_calc)));
    return;
  }

  lcd_rgb((512 * temp_calc), (255 - (510 * temp_calc)), 0);
}

void lcd_rgb(uint8_t val_red, uint8_t val_green, uint8_t val_blue) {
  lcd.write(0xFE); lcd.write(0xD0);

  lcd.write(val_red);
  lcd.write(val_green);
  lcd.write(val_blue);

  delay(10);
}


// Get current fan speed
unsigned int get_rpm() {
  pulse_duration = pulseIn(RPM_IN, LOW);

  if (pulse_duration == 0) return 0;

  unsigned int frequency = 1000000 / pulse_duration;
  return frequency / 8 * 60;
}

void rgb_dance() {
  lcd_rgb(0xFF, 0x00, 0x00); delay(100);
  lcd_rgb(0xFF, 0x00, 0xE0); delay(100);
  lcd_rgb(0xFF, 0x80, 0x00); delay(100);
  lcd_rgb(0xFF, 0xFF, 0x00); delay(100);
  lcd_rgb(0x00, 0xFF, 0x00); delay(100);
  lcd_rgb(0x00, 0xFF, 0xFF); delay(100);
  lcd_rgb(0x00, 0x80, 0xFF); delay(100);
  lcd_rgb(0x7B, 0x46, 0xE8); delay(100);
}


void setup() {
  // Wait for serial connection when debugging
  if (DBG) {
    Serial.begin(115200);
    while (!Serial) {}
  }

  DEBUGLN("[INIT] LCD control");
  setup_lcd();
  lcd_rgb(0xFF, 0x00, 0x00); delay(100);

  DEBUGLN("[INIT] Built-in LED");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  lcd_rgb(0x00, 0xFF, 0x00); delay(100);

  DEBUGLN("[INIT] Fan tachometer");
  pinMode(RPM_IN, INPUT);
  digitalWrite(RPM_IN, HIGH);
  lcd_rgb(0x00, 0x00, 0xFF); delay(100);

  DEBUGLN("[INIT] Fan control");
  pinMode(PWM_OUT, OUTPUT);
  analogWrite(PWM_OUT, 250);
  lcd_rgb(0xFF, 0xFF, 0x00); delay(100);

  DEBUGLN("[INIT] PID control");

  // Setup the PID to work with our settings
  fanPID.SetSampleTime(WAIT);
  fanPID.SetOutputLimits(DUTY_MIN, DUTY_MAX);
  fanPID.SetMode(AUTOMATIC);
  lcd_rgb(0x00, 0xFF, 0xFF); delay(100);

  rgb_dance();
  rgb_dance();
  rgb_dance();

  lcd_rgb(0xFF, 0x00, 0x20);
  delay(500);

  DEBUGLN("[INIT] Complete");

  time_prev = millis();
}

void loop() {
  signed long time_now = millis();

  unsigned int time_diff = time_now - time_prev;

  if (time_diff < WAIT) {
    // Turn on LED
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }


  // Update fan speed
  fan_speed = get_rpm();

  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);

  // Read temperature sensor
  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i = 0; i < SAMPLE_COUNT; i++) {
    samples[i] = analogRead(TEMP_IN);
    delay(10);
  }

  // average all the samples out
  average = 0;
  for (i = 0; i < SAMPLE_COUNT; i++) {
    average += samples[i];
  }
  average /= SAMPLE_COUNT;

  float voltage = (average * 5.000) / 1024.000;
  temp_c = (voltage - 0.470) * 100;

  // Calculate Fahrenheit
  temp_f = (temp_c * 1.8) + 32;

  // Round values
  temp_c = round(temp_c);
  temp_f = round(temp_f);

  // Compute PID fan control and write PWM value out to fan
  fanPID.Compute();
  analogWrite(PWM_OUT, duty);

  // Calculate/format fan duty cycle
  unsigned int fan_duty = map(round(duty), 0, 255, 0, 100);

  // Debug output
  DEBUG("Temp: ");
  DEBUG(temp_c);
  DEBUG("°C/");
  DEBUG(temp_f);
  DEBUG("°F - Power: ");
  DEBUG(fan_duty);
  DEBUG("% - ");

  DEBUG(fan_speed);
  DEBUG("rpm ");
  DEBUGLN(pulse_duration);


  String lcd_string1 = "";
  lcd_string1.concat((uint8_t) temp_c);
  lcd_string1.concat("C      ");

  if (fan_speed < 1000) lcd_string1.concat(" ");
  if (fan_speed < 100)  lcd_string1.concat(" ");
  if (fan_speed < 10)   lcd_string1.concat(" ");

  lcd_string1.concat(fan_speed);
  lcd_string1.concat("rpm");

  String lcd_string2 = "LEDFan 3.0  ";

  if (fan_duty < 100) lcd_string2.concat(" ");
  if (fan_duty < 10)  lcd_string2.concat(" ");

  lcd_string2.concat(fan_duty);
  lcd_string2.concat("%");

  lcd_text(lcd_string2, lcd_string1);
  lcd_rgb_calc();


  // Update time placeholder
  time_prev = time_now;
}
