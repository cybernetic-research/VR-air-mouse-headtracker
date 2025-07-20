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
#define MOUSE_REPORT_ID 1
/*
uint8_t const hidReportDescriptor[] = {
  HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
  HID_USAGE(HID_USAGE_DESKTOP_MOUSE),
  HID_COLLECTION(HID_COLLECTION_APPLICATION),
  HID_REPORT_ID(1)
  HID_USAGE(HID_USAGE_DESKTOP_POINTER),
  HID_COLLECTION(HID_COLLECTION_PHYSICAL),
  HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON),
  HID_USAGE_MIN(1),
  HID_USAGE_MAX(3),
  HID_LOGICAL_MIN(0),
  HID_LOGICAL_MAX(1),
  // buttons 
  HID_REPORT_COUNT(3),
  HID_REPORT_SIZE(1),AXIS_SIGN_NEG, AXIS_
  HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
  // padding 
  HID_REPORT_COUNT(1),
  HID_REPORT_SIZE(5),
  HID_INPUT(HID_CONSTANT),
  HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
  // X, Y position [0, 32767] 
  HID_USAGE(HID_USAGE_DESKTOP_X),
  HID_USAGE(HID_USAGE_DESKTOP_Y),
  HID_LOGICAL_MIN_N(0x0000, 2),
  HID_LOGICAL_MAX_N(0x7fff, 2),
  HID_REPORT_COUNT(2),
  HID_REPORT_SIZE(16),
  HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
  HID_COLLECTION_END,
  HID_COLLECTION_END
};
*/


NimBLEHIDDevice* hid;
NimBLECharacteristic* inputMouse;
bool connected = false;
class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
    Serial.println("BLE HID connected");
    connected = true;
  }

  void onDisconnect(NimBLEServer* pServer) {
    Serial.println("BLE HID disconnected");
    connected = false;
    NimBLEDevice::getAdvertising()->start();
  }
};

const uint8_t hidReportDescriptor[] = {
  0x05, 0x01,       // Usage Page (Generic Desktop)
  0x09, 0x02,       // Usage (Mouse)
  0xA1, 0x01,       // Collection (Application)
    0x09, 0x01,     // Usage (Pointer)
    0xA1, 0x00,     // Collection (Physical)
      0x09, 0x30,   // Usage (X)
      0x09, 0x31,   // Usage (Y)
      0x15, 0x81,   // Logical Min (-127)
      0x25, 0x7F,   // Logical Max (127)
      0x75, 0x08,   // Report Size (8)
      0x95, 0x02,   // Report Count (2)
      0x81, 0x06,   // Input (Data, Variable, Relative)
    0xC0,
  0xC0
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
  
  inputMouse = hid->getInputReport(0); // Report ID 0
  hid->startServices();
  sendRelativeMouse(0,0);
  delay(50);


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

void CheckConnectionstatus()
{
  static bool lastConnected = false;
 
  
  if (connected != lastConnected) {
    lastConnected = connected;
    Serial.printf("BLE conenction %d\n",connected);
  }
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  CheckConnectionstatus();
  UpdateMouse(&event);
  /* Display the floating point data */
  delay(50);
}

void sendRelativeMouse(int8_t dx, int8_t dy) {
  int8_t mouseData[2] = {dx, dy};
  inputMouse->setValue(reinterpret_cast<const uint8_t*>(mouseData), sizeof(mouseData));
  inputMouse->notify();
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
    if (abs(moveX) > 1 || abs(moveY) > 1) {
      sendRelativeMouse(moveX,moveY);
    Serial.printf("connected: %d Mouse Report: X=%d Y=%d\n", connected, moveX, moveY);
  }
  // Update previous values
  lastYaw = yaw;
  lastPitch = pitch;
}
