 #include "painlessMesh.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Wire.h>
#include <ADXL345.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
ADXL345 adxl;

#define   MESH_PREFIX     "myServer"
#define   MESH_PASSWORD   "Iamserver"
#define   MESH_PORT       1234
int sda = 19;
int scl = 23;

Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;


void receivedCallback( uint32_t from, String &msg );

size_t logServerId = 0;
// Send message to the logServer every 10 seconds 
Task myLoggingTask(100, TASK_FOREVER, []() {

#if ARDUINOJSON_VERSION_MAJOR==6
        DynamicJsonDocument jsonBuffer(1024);
        JsonObject msg = jsonBuffer.to<JsonObject>();
#else
        DynamicJsonBuffer jsonBuffer;
        JsonObject& msg = jsonBuffer.createObject();
#endif

int x1,y1,z1;  
  adxl.readXYZ(&x1, &y1, &z1); //read the accelerometer values and store them in variables  x,y,z

  double xyz[3];
  const float alpha = 0.5;
  double fXg = 0;
  double fYg = 0;
  double fZg = 0;

  adxl.getAcceleration(xyz);
 
  String ax = String(xyz[0]*9.81);
  String ay = String(xyz[1]*9.81);
  String az = String(xyz[2]*9.81);

  fXg = xyz[0]* alpha + (fXg * (1.0 - alpha));
  fYg = xyz[1]* alpha + (fYg * (1.0 - alpha));
  fZg = xyz[2]* alpha + (fZg * (1.0 - alpha));

  double rolldata = ((atan2(-fYg, fZg)*180.0)/PI);
  double pitchdata = ((atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/PI);
  if (rolldata<0){
    rolldata = rolldata + 360;
    }
  String roll  = String(rolldata);
  String pitch = String(pitchdata);

  
//-----------------------------------------------------------

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
float yaw = heading * 180/M_PI;
 
//Serial.print("Yaw(degrees): ");
//Serial.println(yaw);

//-----------------------------------------------------------
  
  /*
  String msg = "Hello from node ";
  msg += mesh.getNodeId();
  msg += " myFreeMemory: " + String(ESP.getFreeHeap());
  msg += " AccX " + String(ax);
  msg += " AccY " + String(ay);
  msg += " AccZ " + String(az);
  mesh.sendBroadcast(msg);
 */

   //msg["1"] = "A"+String(roll)+","+String(pitch)+","+String(yaw)+"B";
   //msg["NodeID"] = mesh.getNodeId();
   //msg["2"] = "C"+String(roll)+","+String(pitch)+","+String(yaw)+"D";
   msg["3"] = "E"+String(roll)+","+String(pitch)+","+String(yaw)+"F";

    //msg["node"] = "sensor1  ";
    //msg["nodeid"] = mesh.getNodeId();
    //msg["roll"] = "C"+String(roll)+"D";
    //msg["Pitch"] = String(pitch);
    //msg["Yaw"] = String(yaw);


/*
         msg = "Sensor 2";
         msg+= mesh.getNodeId();
         msg+= "Roll: " + String(roll);
         msg+= "Pitch: " + String(pitch);
         msg+= "Yaw: " + String(yaw); 
*/
    String str;
#if ARDUINOJSON_VERSION_MAJOR==6
    serializeJson(msg, str);
#else
    msg.printTo(str);
#endif
    if (logServerId == 0) // If we don't know the logServer yet
        mesh.sendBroadcast(str);
    else
        mesh.sendSingle(logServerId, str);

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
  Wire.begin(sda,scl);
/* Initialise the sensor */
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
//--------------------------------------------------------------------------------


 // mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, 6 );
  mesh.onReceive(&receivedCallback);

  // Add the task to the your scheduler
  userScheduler.addTask(myLoggingTask);
  myLoggingTask.enable();
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
  //delay(50);
}

void receivedCallback( uint32_t from, String &msg ) {
  //Serial.printf("logClient: Received from %u msg=%s\n", from, msg.c_str());

  // Saving logServer
#if ARDUINOJSON_VERSION_MAJOR==6
  DynamicJsonDocument jsonBuffer(1024 + msg.length());
  DeserializationError error = deserializeJson(jsonBuffer, msg);
  if (error) {
    //Serial.printf("DeserializationError\n");
    return;
  }
  JsonObject root = jsonBuffer.as<JsonObject>();
#else
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(msg);
#endif
  if (root.containsKey("topic")) {
      if (String("logServer").equals(root["topic"].as<String>())) {
          // check for on: true or false
          logServerId = root["nodeId"];
          //Serial.printf("logServer detected!!!\n");
      }
     // Serial.printf("Handled from %u msg=%s\n", from, msg.c_str());
  }
}
