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