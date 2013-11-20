#include <Wire.h>
#include <ADXL345.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#define ScaleFor2G 0.0039
#define Threshold 0.5
#define GPSECHO  true


ADXL345 adxl; //variable adxl is an instance of the ADXL345 library
int accLED=7;
int disLED=6;
int sensorPin=14;
//int flexpin=A1;
SoftwareSerial gpsSerial(3,2);
SoftwareSerial gsmSerial(4,5);
Adafruit_GPS GPS(&gpsSerial);

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

boolean smsSent = false;

void setup(){
  /*pinMode(3,INPUT);
  pinMode(4,INPUT);
  pinMode(2,OUTPUT);
  pinMode(5,OUTPUT);*/
  
  Serial.begin(9600);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);


  adxl.powerOn();
  //if(adxl.EnsureConnected())
  pinMode(accLED,OUTPUT);
  pinMode(disLED,OUTPUT);
    delay(1000);
  gpsSerial.println(PMTK_Q_RELEASE);
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // for debugging
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

void loop(){
  boolean accCrashed = false;
  boolean disCrashed = false;
  
  int x, y, z; 
  float scaledX, scaledY, scaledZ;
  int sensor=analogRead(sensorPin);

//compute acceleration   
  adxl.readAccel(&x, &y,&z);
  scaledX = (float)x * ScaleFor2G;
  scaledY = (float)y * ScaleFor2G;
  scaledZ = (float)z * ScaleFor2G;

  digitalWrite(accLED, LOW);
  digitalWrite(disLED,LOW);
  if((scaledX>Threshold || scaledX<-Threshold || scaledY>Threshold || scaledY<-Threshold ||scaledZ>1+Threshold || scaledZ<1-Threshold) ){
    digitalWrite(accLED, HIGH);
    accCrashed =  true;
  }
  
  
  //Serial.print("distance sensor= ");
  //Serial.println(sensor);
  if(sensor >400){
    //digitalWrite(disLED,HIGH);
    tone(disLED,6000, 40);
    tone(disLED,1000,40);
    tone(disLED,500,40);
    disCrashed =  true;
  } 
 
  
  if (! usingInterrupt) {
  // read data from the GPS
  char c = GPS.read();

  //if (GPSECHO)
   // if (c) Serial.print(c);
   // delay(10000);
  
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
 //   Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  // approximately every 2 seconds or so, print out the current stats
  if (GPS.fix) 
  {
 //   Serial.print("Location: ");
 //   Serial.print(GPS.latitude, 4); //Serial.print(GPS.lat);
 //   Serial.print(", "); 
 //   Serial.println(GPS.longitude, 4); //Serial.println(GPS.lon);
 //   Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    //delay(9000);
  } 
  if( accCrashed && disCrashed )
  {
    gpsSerial.end();
    delay(5000);
    gsmSerial.begin(9600);
    delay(5000);
    sendTextMessage();
    //smsSent = true;
    delay(10000);
    Serial.println("condition satisfied");
    gsmSerial.end();
    delay(3000);
    
    //start GPS
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);
    useInterrupt(true);
    delay(5000);
    gpsSerial.println(PMTK_Q_RELEASE);
  }  

//  Serial.println(smsSent);
   delay(50);
 
}

void sendTextMessage() {
gsmSerial.print("AT+CMGF=1\r");
delay(100);
gsmSerial.println("AT+CMGS=\"+447852576475\"\r");
delay(100);
gsmSerial.print("Emergency: car crashed\n");

gsmSerial.print("Location: ");
gsmSerial.print(GPS.latitude, 4);

gsmSerial.print(", "); 
gsmSerial.print(GPS.longitude, 4);

gsmSerial.print("\r");
delay(100);
gsmSerial.println((char)26);
//Serial.println("in loop");

}
