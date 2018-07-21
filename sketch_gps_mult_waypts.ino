#include "TinyGPS++.h"
#include <SoftwareSerial.h>

static const int RXPin = 6, TXPin = 7;
static const uint32_t GPSBaud = 38400;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin,TXPin);
int wpt = 0; // define variavel wpt e seta valor igual a zero para ela 
double dest_latitude; // define variavel flutuante para latitude de destino 
double dest_longitude; // define variavel flutuante para longitude de destino */
unsigned long lastUpdateTime = 0;
int WPT_RANGE = 3; // define alcance que o carro tem que estar (em metros) para avançar para próximo destino
int mA = 10;   
int mB = 12;  



void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  pinMode(mA,OUTPUT);
  pinMode(mB,OUTPUT);
}

void loop() {
  WayPts();
  // Enquanto chegarem dados no GPS,
  // enviar para o tratamento via biblio TinyGPS++ 
  while (ss.available() > 0)
    gps.encode(ss.read());

  // Update de coordenadas a cada 1 segundos.
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
    Serial.print("Ang. atual: "); Serial.println(atan2(gps.location.lat(), gps.location.lng())*180/M_PI);

   /*
    // Se distância do veiculo é menor que X metros, fim do percurso
    if (distanceToDestination <= 5.0)
    {
      Serial.println("Fim do percurso!");
      para();
      exit(1);
    }
*/

    Serial.print("DISTANCIA: ");
    Serial.print(distanceToDestination);
    Serial.println(" metros restantes.");
    Serial.print("INSTRUCAO: ");

    // Se parado, indicar direção onde ir.
    if (gps.speed.kmph() < 0.5)
    {
      Serial.print("Head ");
      Serial.print(directionToDestination);
      Serial.println(".");
      //return;
    }

    if (courseChangeNeeded >= 345 || courseChangeNeeded < 15)
    {
      Serial.println("Continue em frente!");
      drive_forward();
    }
    else if (courseChangeNeeded >= 315 && courseChangeNeeded < 345)
    {
      Serial.println("Estercar ligeiramente para esquerda.");
      left();
    }
    else if (courseChangeNeeded >= 15 && courseChangeNeeded < 45)
    {
      Serial.println("Estercar ligeiramente para direita.");
      right();
    }
    else if (courseChangeNeeded >= 255 && courseChangeNeeded < 315)
    {
      Serial.println("Vire à esquerda.");
      left();
    }
    else if (courseChangeNeeded >= 45 && courseChangeNeeded < 105)
    {
      Serial.println("Vire à direita.");
      right();
    }
    else
    {
      Serial.println("Vire 180 graus.");
      right();
    }
  }
}

void WayPts () {
/* DEFINIR COORDENADAS DE GPS AQUI */
switch(wpt) {
    case 0:
      dest_latitude = -21.981028;
      dest_longitude = -47.878767;
      break;
    case 1:
      dest_latitude = -21.980925;
      dest_longitude = -47.878908;
      break;
    case 2:
      dest_latitude = -21.980731;
      dest_longitude = -47.878936;
      break;
    case 3:
      dest_latitude = -21.980567;
      dest_longitude = -47.878969;
      break;
    case 4:
      dest_latitude = -21.980339;
      dest_longitude = -47.878975;
      break;
    case 5:
      dest_latitude = -21.980272;
      dest_longitude = -47.879039;
      break;
    case 6:
      dest_latitude = -21.980253;
      dest_longitude = -47.879411;
      break;
    case 7:
      dest_latitude = -21.980339;
      dest_longitude = -47.879514;
      break;
    case 8:
      dest_latitude = -21.980594;
      dest_longitude = -47.879511;
      break;
    default:
      dest_latitude = -21.981028;
      dest_longitude = -47.878767; 
      break;
  }
double distanceToDestination = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), dest_latitude, dest_longitude);
  if (distanceToDestination < WPT_RANGE)
  {
    wpt++; // IMPORTANTE - muda para proxima coordenada 
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
  delay(400);
  digitalWrite(mA, LOW); 
  digitalWrite(mB, LOW);
  delay(600);
}
void left(){
  digitalWrite(mA, LOW);
  digitalWrite(mB, HIGH);
  delay(400);
  digitalWrite(mA, LOW); 
  digitalWrite(mB, LOW);
  delay(600);
}



