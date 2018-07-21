#include "TinyGPS++.h"
#include <SoftwareSerial.h>

static const int RXPin = 6, TXPin = 7;
static const uint32_t GPSBaud = 38400;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin,TXPin);
#define dest_latitude -21.981028
#define dest_longitude -47.878767
unsigned long lastUpdateTime = 0;
int mA = 10;   
int mB = 12;  
float dir;
   
void setup() {
  Serial.begin(115000);
  ss.begin(GPSBaud);
  pinMode(mA,OUTPUT);
  pinMode(mB,OUTPUT);
}

void loop() {
  // Enquanto chegarem dados no GPS,
  // enviar para o tratamento via biblio TinyGPS++ 
  while (ss.available() > 0)
    gps.encode(ss.read());

  // Update de coordenadas a cada 1 seg.
  if (millis() - lastUpdateTime >= 1000)
  {
    lastUpdateTime = millis();
    Serial.println();

    // Estabelecendo status atual
    double distanceToDestination = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), dest_latitude, dest_longitude);
    double courseToDestination = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), dest_latitude, dest_longitude);
    const char *directionToDestination = TinyGPSPlus::cardinal(courseToDestination);
    int courseChangeNeeded = (int)(360 + courseToDestination - gps.course.deg()) % 360;

    // debug
    Serial.print("DEBUG: Course2Dest: ");
    Serial.print(courseToDestination);
    Serial.print("  CurCourse: ");
    Serial.print(gps.course.deg());
    Serial.print("  Dir2Dest: ");
    Serial.print(directionToDestination);
    Serial.print("  RelCourse: ");
    Serial.print(courseChangeNeeded);
    Serial.print("  CurSpd: ");
    Serial.println(gps.speed.kmph());

    float l = gps.location.lat();
    Serial.println();
    Serial.print("Lat: "); Serial.print(l,6); Serial.print("  Lon: "); Serial.println(gps.location.lng());
    Serial.print("current Angle: "); Serial.println(atan2(gps.location.lat(), gps.location.lng())*180/M_PI);

    // Within 5 meters of destination?  We're here!
    if (distanceToDestination <= 5.0)
    {
      Serial.println("CONGRATULATIONS: You've arrived!");
      para();
      exit(1);
    }

    Serial.print("DISTANCE: ");
    Serial.print(distanceToDestination);
    Serial.println(" meters to go.");
    Serial.print("INSTRUCTION: ");

    // Standing still? Just indicate which direction to go.
    if (gps.speed.kmph() < 2.0)
    {
      Serial.print("Head ");
      Serial.print(directionToDestination);
      Serial.println(".");
      //return;
    }

    if (courseChangeNeeded >= 345 || courseChangeNeeded < 15)
    {
      Serial.println("Keep on straight ahead!");
      drive_forward();
    }
    else if (courseChangeNeeded >= 315 && courseChangeNeeded < 345)
    {
      Serial.println("Veer slightly to the left.");
      left();
    }
    else if (courseChangeNeeded >= 15 && courseChangeNeeded < 45)
    {
      Serial.println("Veer slightly to the right.");
      right();
    }
    else if (courseChangeNeeded >= 255 && courseChangeNeeded < 315)
    {
      Serial.println("Turn to the left.");
      left();
    }
    else if (courseChangeNeeded >= 45 && courseChangeNeeded < 105)
    {
      Serial.println("Turn to the right.");
      right();
    }
    else
    {
      Serial.println("Turn completely around.");
      right();
    }
  }
}



void drive_forward(){
digitalWrite(mA, LOW); 
digitalWrite(mB, LOW);
delay(25);
}

void para(){
digitalWrite(mA, HIGH); 
digitalWrite(mB, HIGH);
delay(25);
}

void right(){
  digitalWrite(mA, HIGH);
  digitalWrite(mB, LOW);
  delay(25);
}
void left(){
  digitalWrite(mA, LOW);
  digitalWrite(mB, HIGH);
  drive_forward();
  delay(25);
}



