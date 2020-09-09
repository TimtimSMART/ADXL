#include "painlessMesh.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Wire.h>
#include <ADXL345.h>
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)

int speakerOut = 17;
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it

#endif
BluetoothSerial SerialBT;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
ADXL345 adxl;

#define   MESH_PREFIX     "myServer"
#define   MESH_PASSWORD   "Iamserver"
#define   MESH_PORT       1234

int sda = 19;
int scl = 23;

int ind1,ind2,ind3,ind4,ind5; 
String noneed,rollsubs,pitchsubs,yawsubs, noneed1;
int speakerOut = 17;
//int ind6,ind7,ind8,ind9,ind10; 
//String noneed2,rollsubs1,pitchsubs1,yawsubs1, noneed3;  

Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;
// Prototype
void receivedCallback( uint32_t from, String &msg );


// Send my ID every 10 seconds to inform others
Task logServerTask(10000, TASK_FOREVER, []() {
#if ARDUINOJSON_VERSION_MAJOR==6
        DynamicJsonDocument jsonBuffer(1024);
        JsonObject msg = jsonBuffer.to<JsonObject>();
#else
        DynamicJsonBuffer jsonBuffer;
        JsonObject& msg = jsonBuffer.createObject();
#endif
    msg["topic"] = "logServer";
    msg["nodeId"] = mesh.getNodeId();

    String str;
#if ARDUINOJSON_VERSION_MAJOR==6
    serializeJson(msg, str);
#else
    msg.printTo(str);
#endif
    mesh.sendBroadcast(str);

    // log to serial
#if ARDUINOJSON_VERSION_MAJOR==6
    serializeJson(msg, Serial);
#else
    msg.printTo(Serial);
#endif
    Serial.printf("\n");
});

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Timtim");
  Wire.begin(sda,scl);
if(!mag.begin())
{
/* There was a problem detecting the HMC5883 ... check your connections */
Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
while(1);
}
  //--------------------------------------------------------------------------------
adxl.powerOn();

  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
  //look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
 
  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);
 
  //look of tap movement on this axes - 1 == on; 0 == off
  adxl.setTapDetectionOnX(0);
  adxl.setTapDetectionOnY(0);
  adxl.setTapDetectionOnZ(1);
 
  //set values for what is a tap, and what is a double tap (0-255)
  adxl.setTapThreshold(50); //62.5mg per increment
  adxl.setTapDuration(15); //625us per increment
  adxl.setDoubleTapLatency(80); //1.25ms per increment
  adxl.setDoubleTapWindow(200); //1.25ms per increment
 
  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
  //setting all interrupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  //register interrupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);


  pinMode(17, OUTPUT);  
  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE | DEBUG ); // all types on
  //mesh.setDebugMsgTypes( ERROR | CONNECTION | SYNC | S_TIME );  // set before init() so that you can see startup messages
  mesh.setDebugMsgTypes( ERROR | CONNECTION | S_TIME );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, 6 );
  //mesh.init( MESH_PREFIX, MESH_PASSWORD, MESH_PORT, STA_AP, WIFI_AUTH_WPA2_PSK, 6 );
  mesh.onReceive(&receivedCallback);

  mesh.onNewConnection([](size_t nodeId) {
    Serial.printf("New Connection %u\n", nodeId);
  });

  mesh.onDroppedConnection([](size_t nodeId) {
    Serial.printf("Dropped Connection %u\n", nodeId);
  });

  // Add the task to the your scheduler
  userScheduler.addTask(logServerTask);
  logServerTask.enable();
}

void loop() {

int x1,y1,z1;  
  adxl.readXYZ(&x1, &y1, &z1); //read the accelerometer values and store them in variables  x,y,z

  double xyz[3];
  const float alpha = 0.5;
  double fXg = 0;
  double fYg = 0;
  double fZg = 0;

  adxl.getAcceleration(xyz);
 
  //String ax = String(xyz[0]*9.81);
  //String ay = String(xyz[1]*9.81);
  //String az = String(xyz[2]*9.81);

  fXg = xyz[0]* alpha + (fXg * (1.0 - alpha));
  fYg = xyz[1]* alpha + (fYg * (1.0 - alpha));
  fZg = xyz[2]* alpha + (fZg * (1.0 - alpha));

  int rolldata = ((atan2(-fYg, fZg)*180.0)/PI);
  int pitchdata = ((atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/PI);
  if (rolldata<0){
    rolldata = rolldata + 360;
    }
  String roll  = String(rolldata);
  String pitch = String(pitchdata);


/* Get a new sensor event */
sensors_event_t event;
mag.getEvent(&event);
 
/* Display the results (magnetic vector values are in micro-Tesla (uT)) */
/*
Serial.print("X: ");
Serial.print(event.magnetic.x);
Serial.print(" ");
Serial.print("Y: ");
Serial.print(event.magnetic.y);
Serial.print(" ");
Serial.print("Z: ");2
Serial.print(event.magnetic.z);
Serial.print(" ");
Serial.println("uT");
*/

float heading = atan2(event.magnetic.z, event.magnetic.x);

// Find yours here: http://www.magnetic-declination.com/

float declinationAngle = 0.276; //15.46deg to 0.276 rad
heading += declinationAngle;
 
// Correct for when signs are reversed.
if(heading < 0)
heading += 2*PI;
 
// Check for wrap due to addition of declination.
if(heading > 2*PI)
heading -= 2*PI;
 
// Convert radians to degrees for readability.
String yaw = String(heading * 180/M_PI);
 
//Serial.print("Yaw(degrees): ");
//Serial.println(yaw);
  
//-----------------------------------------------------------

  //**************************************************************************
  // it will run the user scheduler as well
  int kneex = rollsubs.toInt() - roll.toInt();
  int kneey = pitchsubs.toInt() - pitch.toInt();
  int kneez = yawsubs.toInt() - yaw.toInt();
  Serial.print("KneeX : ");
  Serial.println(kneex);
  Serial.print("KneeY : ");
  Serial.println(kneey);
  Serial.print("KneeZ : ");
  Serial.println(kneez);

  SerialBT.print("KneeX : ");
  SerialBT.println(kneex);
  SerialBT.print("KneeY : ");
  SerialBT.println(kneey);
  SerialBT.print("KneeZ : ");
  SerialBT.println(kneez);

  if(abs(kneex)<40 && abs(kneey)>15 && abs(kneez)>30){
    Serial.println("*********************************************");
    digitalWrite(17, HIGH);
    }else{
       digitalWrite(17, LOW );
      }

  
  //delay(100);
  mesh.update();
}

void receivedCallback( uint32_t from, String &msg ) {
  //Serial.printf("logServer: Received from %u msg=%s\n", from, msg.c_str());
  //Serial.printf("%u %s\n", from, msg.c_str());
  Serial.printf("%s\n", msg.c_str());
  
  //if(from==3296781833){
  if(from==3296868969){
    
  ind1 = msg.indexOf('A');
  noneed = msg.substring(0, ind1);
  ind2 = msg.indexOf(',', ind1+1 );
  rollsubs = msg.substring(ind1+1, ind2);
  ind3 = msg.indexOf(',', ind2+1 );
  pitchsubs = msg.substring(ind2+1, ind3);
  ind4 = msg.indexOf('B', ind3+1 );
  yawsubs = msg.substring(ind3+1, ind4);
  ind5 = msg.indexOf('B', ind4+1 );
  noneed1 = msg.substring(ind4+1);
  //Serial.println(rollsubs);
  //Serial.println(pitchsubs);
  //Serial.println(yawsubs);
  }
  /*
  //else if(from==3204660733){
  else if(from==3204662961){
 
  ind6 = msg.indexOf('C');
  noneed2 = msg.substring(0, ind6);
  ind7 = msg.indexOf(',', ind6+1 );
  rollsubs1 = msg.substring(ind6+1, ind7);
  ind8 = msg.indexOf(',', ind7+1 );
  pitchsubs1 = msg.substring(ind7+1, ind8);
  ind9 = msg.indexOf('D', ind8+1 );
  yawsubs1 = msg.substring(ind8+1, ind9);
  ind10 = msg.indexOf('D', ind9+1 );
  noneed3 = msg.substring(ind9+1);
  //Serial.println(rollsubs1);
  //Serial.println(pitchsubs1);
  //Serial.println(yawsubs1);
  }
  */
/*
  int kneex = rollsubs.toInt() - rollsubs1.toInt();
  Serial.print("KneeX : ");
  Serial.println(kneex);
  */

}
