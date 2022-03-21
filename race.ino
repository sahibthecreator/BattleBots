#include "Adafruit_VL53L0X.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/*Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const byte led_gpio = 17; // the PWM pin the LED is attached to
const byte led_gpio1 = 5; // the PWM pin the LED is attached to

const byte back1 = 16; // the PWM pin the LED is attached to
const byte back2 = 18; // the PWM pin the LED is attached to
int leftSensor = 39; 
int rightSensor = 34; 

int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by

// the setup routine runs once when you press reset:
void setup() {
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

  pinMode (leftSensor, INPUT); // sensor pin INPUT
  pinMode (rightSensor, INPUT); // sensor pin INPUT

  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
}

// the loop routine runs over and over again forever:
void loop() {
   VL53L0X_RangingMeasurementData_t measure;
    

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  //displayDistance();



  LineTracking();
  
  /*if(measure.RangeMilliMeter > 150){
    goForward(170);
  }else{
    goBack();
    delay(1200);
    turnRight();
    delay(1000);
    }
    */
  
/*}
int goForward(int speed){
    ledcWrite(0, speed); 
    ledcWrite(1, speed); 
    ledcWrite(2, 0); 
    ledcWrite(3, 0); 
}

int goBack(int speed){
    ledcWrite(0, 0); 
    ledcWrite(1, 0); 
    ledcWrite(2, speed); 
    ledcWrite(3, speed); 
}

int turnLeft(int speed){
    ledcWrite(0, speed); 
    ledcWrite(1, 0); 
    ledcWrite(2, 0); 
    ledcWrite(3, speed); 
}

int turnRight(int speed){
    ledcWrite(0, 0); 
    ledcWrite(1, speed); 
    ledcWrite(2, speed); 
    ledcWrite(3, 0); 
}

int LineTracking(){ 
  int sensorL = analogRead (leftSensor);
  int sensorR = analogRead (rightSensor);
  int vSpeed = 125;        // MAX 255
  int bustSpeed = 255;    // MAX 255 
  int boost_delay = 3000;

  Serial.print(F("Right sensor = "));
  Serial.println((sensorR));
    Serial.print(F("Left sensor = "));
  Serial.println((sensorL));
  
  if(sensorR < 300 && sensorL < 300)
{
  Serial.println("ggogogogogo");

  goForward(vSpeed);
  
  }
  if(sensorR > 300 && sensorL > 300)
{
  Serial.println("boooooost");

  turnRight(bustSpeed);
  delay(boost_delay);
  
  }*/

  //for ESP32 microcontroller
int trigPin = 4:
int echoPin = 0;     

NewPing sonar(trigPin, echoPin);

void setup(){
  Serial.begin(9600);
}

void loop(){
  float distance = sonar.ping_median(5);

  if(distance>400 || distance<2) Serial.println("Out of range");
  else
  {
    Serial.print("Distance: ");
    Serial.print(distance, 1); Serial.println(" cm");
  }
 
  delay(50);
}