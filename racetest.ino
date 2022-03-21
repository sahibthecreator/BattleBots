
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

#include <hcsr04.h>
#include <ESP32Servo.h>

#define TRIG_PIN 14
#define ECHO_PIN 12

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)

HCSR04 hcsr04(TRIG_PIN, ECHO_PIN, 20, 4000);
Servo myservo;  // create servo object to control a servo

const byte led_gpio = 17; // the PWM pin the LED is attached to
const byte led_gpio1 = 5; // the PWM pin the LED is attached to

const byte back1 = 16; // the PWM pin the LED is attached to
const byte back2 = 18; // the PWM pin the LED is attached to

int leftSensor = 39;
int rightSensor = 34;

int servoPin = 19;
int pos = 0;

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

  //------------Servo Motor--------------
  ledcSetup(6, 50, 16); // channel 2, 50 Hz, 16-bit width
  ledcAttachPin(19, 6);   // GPIO 19 assigned to channel 2


  //-----------Line Tracking Sensor----------
  pinMode (leftSensor, INPUT); // sensor pin INPUT
  pinMode (rightSensor, INPUT); // sensor pin INPUT

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }


}

void loop() {

  if (hcsr04.distanceInMillimeters() > 300) {
    Serial.println("Front sensor: ");
    Serial.println(hcsr04.distanceInMillimeters());
    goForward(200);
  }
  else {

    stopCar();
    lookRight();
    int right = hcsr04.distanceInMillimeters();
    Serial.println("right sensor: ");
    Serial.println(hcsr04.distanceInMillimeters());

    lookLeft();
    int left = hcsr04.distanceInMillimeters();
    Serial.println("left sensor: ");
    Serial.println(hcsr04.distanceInMillimeters());

    if (right > left) {
      Serial.println("right MORE: ");
      lookForward();
      while (hcsr04.distanceInMillimeters() >= right) {
        turnRight(200);

      }
    } else {
      Serial.println("LEFT MORE: ");
      lookForward();
      while (hcsr04.distanceInMillimeters() >= left) {
        turnLeft(200);

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

  int lookLeft() {
    ledcWrite(6, 7864);
    delay(500);
  }

  int lookRight() {
    ledcWrite(6, 1638);
    delay(500);
  }

  int lookForward() {
    int value = map(90, 0, 180, 1638, 7864);
    ledcWrite(6, value);
    delay(500);
  }
