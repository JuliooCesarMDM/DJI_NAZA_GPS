#include "DJINAZAGPS.h"

/******************************INFO**********************************/
//AUTOR:JULIO CESAR MATIAS
//DESENVOLVIDO OFICIALMENTE PARA A CONTROLADORA DE VOO JCFLIGHT
//DATA:JULHO DE 2020
/********************************************************************/

void setup() {
  Serial.begin(115200);  //BAUD-RATE DO SERIAL MONITOR
  Serial1.begin(115200); //BAUD-RATE DO GPS
  pinMode(13, OUTPUT);
}

boolean OnePrint = false;
void loop() {
  //PARA MICROC ATMEGA 328 PODE SE USAR A SOFTWARE SERIAL PARA A LEITURA DO GPS
  uint8_t GPSSerialDataRead = Serial1.read(); //GPS CONECTADO NO TX1 E RX1 DO MEGA 2560

  if (DjiNazaGpsNewFrame(GPSSerialDataRead)) {
    if (!OnePrint)Serial.println("GPS OK!");
    OnePrint = true;
  } else Serial.println("SEM INFO!"), OnePrint = false;

  if (!OnePrint)return; //APENAS PARA NÃO PRINTAR VARIOS ZEROS JUNTO COM OS SERIAL PRINTLN ACIMA

  if (DJINaza_Num_Sat >= 5 && millis() % 1000 > 500)digitalWrite(13, HIGH); else digitalWrite(13, LOW);

  Serial.print(DJINaza_Num_Sat);
  Serial.print(" ");

  Serial.print(DJINaza_Latitude);      //COORDENADA LATITUDE
  Serial.print(" ");

  Serial.print(DJINaza_Longitude);     //COORDENADA LONGITUDE
  Serial.print(" ");

  Serial.print(DJINaza_Altitude);      //ALTITUDE DADA PELO GPS EM CENTIMETROS
  Serial.print(" ");

  Serial.print(DJINaza_HDOP);          //HDOP
  Serial.print(" ");

  Serial.print(DJINaza_Fix_State);     //2D E 3D GPS FIX
  Serial.print(" ");

  Serial.print(DJINaza_Compass_Roll);  //EIXO X DO MAG DO GPS
  Serial.print(" ");

  Serial.print(DJINaza_Compass_Pitch); //EIXO Y DO MAG DO GPS
  Serial.print(" ");

  Serial.print(DJINaza_Compass_Yaw);   //EIXO Z DO MAG DO GPS
  Serial.print(" ");

  Serial.print(DJINaza_GroundCourse);  //GROUND COURSE EM GRAUS
  Serial.print(" ");

  Serial.print(DJINaza_GroundSpeed);   //GROUND SPEED EM CM/S
  Serial.print(" ");

  Serial.println();
}
