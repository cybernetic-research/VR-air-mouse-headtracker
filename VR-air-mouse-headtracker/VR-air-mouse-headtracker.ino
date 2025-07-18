//vr head tracker
//chip ESP32 WROVER 

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEHIDDevice.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55,0x29);
float lastYaw = 0;
float lastPitch = 0;

const uint8_t hidReportDescriptor[] = {
  0x05, 0x01,       // Usage Page (Generic Desktop)
  0x09, 0x02,       // Usage (Mouse)
  0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,     // Report ID (1)
    0x09, 0x01,     // Usage (Pointer)
    0xA1, 0x00,     // Collection (Physical)
      0x05, 0x09,   // Usage Page (Buttons)
      0x19, 0x01,
      0x29, 0x03,
      0x15, 0x00,
      0x25, 0x01,
      0x95, 0x03,
      0x75, 0x01,
      0x81, 0x02,   // Button inputs
      0x95, 0x01,
      0x75, 0x05,
      0x81, 0x03,   // Padding
      0x05, 0x01,
      0x09, 0x30,
      0x09, 0x31,
      0x15, 0x81,
      0x25, 0x7F,
      0x75, 0x08,
      0x95, 0x02,
      0x81, 0x06,   // X/Y as relative movement
    0xC0,
  0xC0
};


NimBLEHIDDevice* hid;
NimBLECharacteristic* inputMouse;

class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer) {
    Serial.println("Connected");
  }
  void onDisconnect(NimBLEServer* pServer) {
    Serial.println("Disconnected — restarting advertising");
    NimBLEDevice::getAdvertising()->start();
  }
};

void setupMouseHID() {
  NimBLEDevice::init("VR HeadMouse");
  NimBLEServer* pServer = NimBLEDevice::createServer();
  hid = new NimBLEHIDDevice(pServer);

  // Basic HID descriptor — you can customize this further if needed
  hid->setManufacturer("CyberneticResearch");
  hid->setBatteryLevel(100);
  hid->setReportMap((uint8_t*)hidReportDescriptor, sizeof(hidReportDescriptor));
  hid->setHidInfo(0x00, 0x01); // HID version 1.0
  hid->setPnp(0x02, 0xE502, 0xA111, 0x0210); // PnP info
  NimBLEDevice::setSecurityAuth(false, false, false); // Enable bonding
  
  inputMouse = hid->getInputReport(1); // Report ID 0
  hid->startServices();

  delay(50);
  int8_t mouseData[4] = {1, 0, 0, 0};
  inputMouse->setValue((uint8_t*)mouseData, sizeof(mouseData));
  inputMouse->notify();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setAppearance(HID_MOUSE);
  pAdvertising->addServiceUUID(hid->getHidService()->getUUID());
  pAdvertising->start();
  pServer->setCallbacks(new MyServerCallbacks());
}

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);   
  bno.setExtCrystalUse(true);
  Wire.begin(21,22);
  setupMouseHID();
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  UpdateMouse(&event);
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  delay(50);
}

void UpdateMouse(sensors_event_t *event)
{
  float yaw = event->orientation.z;
  float pitch = event->orientation.y;
  float dx = yaw - lastYaw;
  float dy = pitch - lastPitch;
  // Optional sensitivity scaling
  int sensitivity = 2;
  int moveX = int(dx * sensitivity);
  int moveY = int(dy * sensitivity);
  int8_t mouseData[4] = {1, 0, moveX, moveY};
    if (abs(moveX) > 1 || abs(moveY) > 1) {
    inputMouse->setValue((uint8_t*)mouseData, sizeof(mouseData));
    inputMouse->notify();
  }
  // Update previous values
  lastYaw = yaw;
  lastPitch = pitch;
}
