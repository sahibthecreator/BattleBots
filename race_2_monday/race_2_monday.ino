#include "Adafruit_VL53L0X.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <hcsr04.h>
#include <ESP32Servo.h>
#include <hcsr04.h>

#define TRIG_PIN 25
#define ECHO_PIN 26


Adafruit_VL53L0X lox = Adafruit_VL53L0X();
HCSR04 left_dist_sensor(TRIG_PIN, ECHO_PIN, 20, 4000);

const byte led_gpio = 17; // the PWM pin the LED is attached to
const byte led_gpio1 = 5; // the PWM pin the LED is attached to

const byte back1 = 16; // the PWM pin the LED is attached to
const byte back2 = 18; // the PWM pin the LED is attached to


long duration;
float distanceCm;

bool checked = false;

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);

  ledcAttachPin(led_gpio, 0); // assign a led pins to a channel
  ledcAttachPin(led_gpio1, 1); // assign a led pins to a channel
  ledcAttachPin(back1, 2); // assign a led pins to a channel
  ledcAttachPin(back2, 3); // assign a led pins to a channel

  // Initialize channels
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(0, 4000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(1, 4000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 4000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(3, 4000, 8); // 12 kHz PWM, 8-bit resolution



  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  //------Distance Sensor Test----------

    if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
    }
}

// the loop routine runs over and over again forever:
void loop() {



  //LineTracking()

    
    if (measure.RangeMilliMeter > 200) {
        goForward(180);
    } else {
        stopCar();
        if (getDistanceL() < 100) {
        while (measure.RangeMilliMeter < 250 && getDistanceL() > 100) { // rotation algorithm
            turnRight(190);
            delay(50);
        }
        } else {
        while (getDistanceF() < 250 ) { // rotation algorithm
            turnLeft(190);
            delay(50);
        }
        }
    }


  }


int goForward(int speed) {
  ledcWrite(0, speed);
  ledcWrite(1, speed);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

int goBack(int speed) {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, speed);
  ledcWrite(3, speed);
}

int turnLeft(int speed) {
  ledcWrite(0, speed);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, speed);
}

int turnRight(int speed) {
  ledcWrite(0, 0);
  ledcWrite(1, speed);
  ledcWrite(2, speed);
  ledcWrite(3, 0);
}

int stopCar() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}
 
int getDistanceL() {
  return left_dist_sensor.distanceInMillimeters();
}


int LineTracking(){ 
  int sensorL = analogRead (leftSensor);
  int sensorR = analogRead (rightSensor);
  int bustSpeed = 255;    // MAX 255 
  int boost_delay = 3000;
  
  if(sensorR > 300 && sensorL > 300)
{
  Serial.println("boooooost");

  turnRight(bustSpeed);
  delay(boost_delay);
  
  }

int displayDistance() {
  VL53L0X_RangingMeasurementData_t measure;


  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println(measure.RangeMilliMeter);
  display.display();
}
