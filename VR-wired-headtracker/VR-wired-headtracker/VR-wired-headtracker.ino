#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Mouse.h>   

Adafruit_BNO055 bno = Adafruit_BNO055(55,0x29);
float lastYaw = 0;
float lastPitch = 0;
int enableMotion = 0;
#define USE_IMU 1

void InitIMU()
{
  if(USE_IMU)
  {
    Serial.println("Orientation Sensor Test"); Serial.println("");  
    /* Initialise the sensor */
    if(!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
    else
    {
    }
    delay(3000);   
    bno.setExtCrystalUse(true);
  }
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);   // UART for GPS
  Serial.print("test0");
  Serial1.print("test1");
  InitIMU();
  pinMode(A3, INPUT_PULLUP); // Enables internal pull-up
}

int sensitivity  = 20;

#define GPS_BUFFER_SIZE 128
char gpsBuffer[GPS_BUFFER_SIZE];
uint8_t gpsIndex = 0;

void HandleGPS()
{
  while (Serial1.available()) {
    char c = Serial1.read();
    
    if (gpsIndex < GPS_BUFFER_SIZE - 1) {
      gpsBuffer[gpsIndex++] = c;
    }

    if (c == '\n') {
      gpsBuffer[gpsIndex] = '\0'; // Null-terminate
      Serial.print(gpsBuffer);    // Dump full GPS line
      Serial.print("$IMU,");
      Serial.print(lastYaw);
      Serial.print(",");
      Serial.print(lastPitch),
      Serial.print(",");
      Serial.print(enableMotion);
      Serial.print(",");
      Serial.println(sensitivity);
      gpsIndex = 0;               // Reset buffer
    }
  }
}

#define CMD_BUFFER_SIZE 32
char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;
void HandleSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();

    if (cmdIndex < CMD_BUFFER_SIZE - 1) {
      cmdBuffer[cmdIndex++] = c;
    }

    if (c == '\n') {
      cmdBuffer[cmdIndex] = '\0';

      if (strncmp(cmdBuffer, "sens=", 5) == 0) {
        int newSens = atoi(&cmdBuffer[5]);
        if (newSens > 0 && newSens < 100) {  // reasonable bounds
          sensitivity = newSens;
          Serial.print("Sensitivity set to: ");
          Serial.println(sensitivity);
        }
      }

      // You can add more commands like "center\n" here.

      cmdIndex = 0; // Reset for next line
    }
  }
}

void HandleMouse()
{
  if(USE_IMU)
  {
    /* Get a new sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);
    UpdateMouse(&event);
  } 
}

void loop() {
  HandleSerialCommand();  // Listen to USB host commands
  HandleMouse();          // Move cursor based on IMU
  HandleGPS();            // Stream GPS + IMU telemetry
  delay(10);
}



float unwrap(float current, float last) {
  float delta = current - last;
  if (delta > 180) delta -= 360;
  if (delta < -180) delta += 360;
  return delta;
}

void UpdateMouse(sensors_event_t *event)
{
  enableMotion = digitalRead(A3); // HIGH or LOW
  float pitch = event->orientation.y; // Using roll instead
  float yaw = event->orientation.x;   // Treat pitch as yaw, for rotated board
  float dx = unwrap(yaw, lastYaw);
  float dy = unwrap(pitch, lastPitch);  
  int x = int(dx * sensitivity);
  int y = int(dy * sensitivity);
  if (enableMotion == HIGH) 
  {
    Mouse.move(x, y);
  }
  // Update previous values
  lastYaw = yaw;
  lastPitch = pitch;
}
