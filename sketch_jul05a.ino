    /////////////////////////////////////////////////////////////////
   //         ESP32 Web Server Project  v1.00                     //
  //       Get the latest version of the code here:              //
 //         http://educ8s.tv/esp32-web-server                   //
/////////////////////////////////////////////////////////////////


#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>  //https://github.com/bbx10/WebServer_tng
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <stdio.h>
#include <math.h>

WebServer server ( 80 );

const char* ssid     = "TP-Link_DD90";
const char* password = "tuantrungnguyen";
#define ALTITUDE 216.0 // Altitude in Sparta, Greece

#define BME280_ADDRESS 0x76  //If the sensor does not work, try the 0x77 address as well
#define BNO055_SAMPLERATE_DELAY_MS (1000)
float a=0;
float b=0;
float v;
float m=0;
float n=0;
int LEDPIN = 32;

float temperature = 0;
float humidity = 0;
float pressure = 0;

String  ledState = "OFF";

Adafruit_BNO055 bno = Adafruit_BNO055(55); 

void setup() 
{
  pinMode(LEDPIN, OUTPUT);
  
  Serial.begin(9600);

  initSensor();

  connectToWifi();

  beginServer();
}

void loop() {
 
 server.handleClient();
imu::Vector<3> acc =bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

sensors_event_t event; 
bno.getEvent(&event);
  
  /* Display the floating point data */

 
//Serial.print(acc.x());
//Serial.print(" ");
//Serial.print(acc.y());
//Serial.print(" ");
//Serial.println(acc.z());
 
if ((abs(acc.x()-m)<0.07 )) {
  a=0;
} else{
a=a+acc.x()-1.83;
m=acc.x();
}
if (abs(acc.y()-n)<0.07) {
  b=0;
} else{
b=b+acc.y()+0.35;
n=acc.y();
}
v=sqrt(a*a+b*b);
 delay(1000);
 
}

void connectToWifi()
{
  WiFi.enableSTA(true);
  
  delay(2000);

  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void beginServer()
{
  server.on ( "/", handleRoot );
  server.begin();
  Serial.println ( "HTTP server started" );
}

void handleRoot(){ 
  if ( server.hasArg("LED") ) {
    handleSubmit();
  } else {
    server.send ( 200, "text/html", getPage() );
  }  
}

void initSensor()
{
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
}



void handleSubmit() {

  String LEDValue;
  LEDValue = server.arg("LED");
  Serial.println(". "); 
  Serial.print(LEDValue);
  
  if ( LEDValue == "1" ) {
    digitalWrite(LEDPIN, HIGH);
    ledState = "On";
    server.send ( 200, "text/html", getPage() );
  }
  else if( LEDValue == "0" ) 
  {
    digitalWrite(LEDPIN, LOW);
    ledState = "Off";
    server.send ( 200, "text/html", getPage() );
  } else 
  {
    Serial.println("Error Led Value");
  }
}

String getPage(){
  String page = "<html lang=en-EN><head><meta http-equiv='refresh' content='60'/>";
  page += "<title>ESP32 WebServer </title>";
  page += "<style> body { background-color: #fffff; font-family: Arial, Helvetica, Sans-Serif; Color: #000000; }</style>";
  page += "</head><body><h1>ESP32 WebServer</h1>";
  page += "<h3>TargetTracking</h3>";
  page += "<ul><li>Acc.x: ";
  page += a;
  page += "m/s2</li>";
  page += "<li>Acc.y: ";
  page += b;
  page += "m/s2</li>";
  page += "<li>Velocity: ";
  page += v;
  page += " m/s</li></ul>";
  page += "<h3>GPIO</h3>";
  page += "<form action='/' method='POST'>";
  page += "<ul><li>LED";
  page += "";
  page += "<INPUT type='radio' name='LED' value='1'>ON";
  page += "<INPUT type='radio' name='LED' value='0'>OFF</li></ul>";
  page += "<INPUT type='submit' value='Submit'>";

  page += "</body></html>";
  return page;
}
