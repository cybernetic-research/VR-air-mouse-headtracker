#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Mouse.h>   

Adafruit_BNO055 bno = Adafruit_BNO055(55,0x29);
float lastYaw = 0;
float lastPitch = 0;
void setup() {
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(3000);   
  bno.setExtCrystalUse(true);
}

void loop() {
  // put your main code here, to run repeatedly:
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  UpdateMouse(&event);
  delay(100);
}

void UpdateMouse(sensors_event_t *event)
{
  float yaw = event->orientation.z;
  float pitch = event->orientation.y;
  float dx = yaw - lastYaw;
  float dy = pitch - lastPitch;
  // Optional sensitivity scaling
  int sensitivity = 2;
  int x = int(dx * sensitivity);
  int y = int(dy * sensitivity);
    if (abs(x) > 1 || abs(y) > 1) {
      Mouse.move(x, y); 
  }
  // Update previous values
  lastYaw = yaw;
  lastPitch = pitch;
}
