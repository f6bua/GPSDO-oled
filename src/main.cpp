#include <Arduino.h>
/******************************************************************************************************************
 *  --- Fout = 100 KHz ( pour afficheur 4 lignes de 16 ou 20 caracteres et afficheur OLED 128x64 )
 *
 * Alain fort F1CJN  November, 11, 2016     /   F1TE  Lucien Serrano  06 Octobre 2021
 * Modifié par F6BUA le 10/01/2022
 ****************************************************************************************************************** 
 * 
 * This program is made :
 * - first to program the Timepulse frequency of an UBLOX GPS NEO-8M module
 * - write on a 4 line LCD : latitude, longitude, Time, number of received satellites and the QRA Locator.
 *
 * La fréquence peut être modifiée en ligne 125/
 * The signal Timepulse est généré à partir d'une horloge à 48 MHz.
 * Seules les facteurs de divisions entiers pairs (2,4,6, 342, ....) présentent un très faible jitter. Par exemple 8MHz est ok, 10 KHz est ok 
 * mais 10 MHz ne l'est pas (facteur de division de 4,8). La fréquence de Timepulse est programmé pour 10 KHz mais peut être modifiée à la ligne 125.
 * 
 * Connections to the GPS board
 * GPS GND connects to Arduino Gnd
 * GPS RX connects to Software Serial pin 3 on the Arduino Nano, via voltage divider : 330 Ohms in serial to 680 Ohms to ground        Pin 4--330--NEO RX--680--GND
 * GPS TX connects to Software Serial pin 4 on the Arduino Nano
 * GPS VCC connects to 5 volts
 * 
 * Connections to the serial LCD 4x16 board
 * Arduino 5V to LCD 5V
 * Arduino GND to LCD GND
 * Arduino A4 to LCD SDA
 * Arduino A5 to CLD SCL
 * 
 * Serial O/P at 9600 ( or 115200 ) is used for test purposes 
 * 
 * Dans la librairie TinyGPSPlus.cpp, il faut modifier : 
 *   #define _GPRMCterm   "GNRMC" // remplace GPRMC 
 *   #define _GPGGAterm   "GNGGA"  // remplace GPGGA 
 * 
 **************************************************************************/

#include "SoftwareSerial.h"
#include <TinyGPSPlus.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
         // 
//#include <Wire.h>                 // Comes with Arduino IDE

//#include "../lib/TinyGPSPlus/TinyGPSPlus.h" 
//#include "../lib/SSD1306Ascii/src/SSD1306Ascii.h"
//#include "../lib/SSD1306Ascii/src/SSD1306AsciiWire.h"

// Librairie et adresse OLED version 1
//************************************
#define I2C_ADDRESS 0x3C  // 0x3C ou 0x78 
#define RST_PIN -1
SSD1306AsciiWire oled;    // 

/*
/ Librairie et adresse OLED version 2
************************************
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#define I2C_ADDRESS 0x3C
#define RST_PIN -1
SSD1306AsciiAvrI2c oled;
*/

boolean gpsStatus[] = {false, false, false, false, false, false, false, false, false};
unsigned long frequency;
byte buf[4];

// Communication GPS
#define RXPIN 4       // vers GPS TX
#define TXPIN 3       // vers GPS RX via pont diviseur 330/680 Ohms pour adapter en 3,3V
#define GPSBAUD 9600  //
SoftwareSerial gpsSerial(RXPIN,TXPIN);    // 4,3
TinyGPSPlus gps;

// Traitement tension boucle de phase
byte LED_loop = 6;                        // I/O Pin 9
byte Port_anal_loop = 0;                  // -- Input A0  
int val_loop;                             // 
int moy_val_loop;                         // 
int old_moy_val_loop;                     //
int seuil_loop = 21;                      // fenêtre de 200mV, 2 fois (5 / 1024) * 21 = 0.1V   
int lock;

//void getgps(TinyGPSPlus &gps);
 float latitude;
 float longitude;
 char locator[10];


// Calcul du QRA locator en mode complet
void calcLocator(char *dst,float lat, float lng )     // Calcule le Locator ...
{
  int lon1, lon2, lon3, lon4, lon5;
  int la1, la2, la3, la4, la5;
  float reste;

  //Les 2 lignes suivantes peuvent être remplacées par des boucles for pour initialiser le tableau
  char alphabet_maj[26]={'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'};
  char alphabet_min[26]={'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z'};
  

 //==============================CALCUL LONGITUDE==================================

  //Un carré en longitude = 20° (20*18 => 360°)
  //La longitude 0 GPS est décalé de 180° (20°*9) de la longitude "0" du MLS
  //Le premier caractère du MLS sera donc A si long est dans l'intervale [-180;-160[
  //Il est nécessaire de rajouter 180° à la valeur de lng pour être bien aligné
  
  lng+=180;

  //Calcul lon1
  lon1=trunc(lng/20);
  reste = fmod(lng,20);

  //Une fois le "Field" calculé, il faut maintenant calculer la position du square dans ce field
  //Dans un field il y a 100 squares (une "matrice" de 10X10)
  //Un field en longitude représentant 20°, un square en longitude représente 2° (20/10)
  //Un field en latitude représentant 10°, un square en latitude représente 1° (10/10)

  lon2 = trunc(reste/2);
  reste = fmod(reste,2);

  //Une fois le "square" calculé, il faut maintenant calculer la position du subsquare dans ce square
  //Dans un square il y a 576 subsquare (une "matrice" de 24*24)
  //Un square en longitude représentant 2°, un subsquare en longitude représente 2/24 ° , afin de garder des calculs simple on va convertir les degrés restant en minute
  //Un degrés = 60 minutes => un square en longitude représente 2° => un square en longitude représente 120minutes => un subsquare en longitude représent 5minutes (120/24)

  //Un square en latitude représentant 1°, un subsquare en latitude représente 1/24 ° , afin de garder des calculs simple on va convertir les degrés restant en minute
  //Un degrés = 60 minutes => un square en latitude représente 1° => un square en latitude représente 60 minutes => un subsquare en latitude représent 2.5 minutes (60/24)

  lon3 = trunc(reste*60/5);
  reste = fmod(reste*60,5);

  //Une fois le "subsquare" calculé, il faut maintenant calculer la position de l'extended square dans ce subsquare
  //Dans un subsquare il y a 100 extend squares (une "matrice" de 10)

  //Un subsquare en longitude représentant 5 minutes, un extended square en longitude représente 5/10 minute , afin de garder des calculs simple on va convertir les minutes restante en secondes
  //Une minutes = 60 secondes => un subsquare en longitude représente 5 minutes => un subsquare en longitude représente 300 secondes => un extended square en longitude représent 30s (300/10)

  //Un subsquare en latitude représentant 1°, un extended quare en latitude représente 1/10 minute , afin de garder des calculs simple on va convertir les minutes restantes en secondes
  //Une minute = 60 secondes => un square en latitude représente 2.5 minutes => un square en latitude représente 150 minutes => un extended square en latitude représent 15 minutes (150/10)

  lon4 = trunc(reste*60/30);
  reste = fmod(reste*60,30);

  //Une fois l extended square calculé, on peut calculer un subsquare dans cet extended subsquare
  //Dans un extended square il y a 576 subsquares (une "matrice" de 24X24)

  //Un extended square en longitude représentant 30s, un subsquare dans cet extended square en longitude représente 30/24 secondes , afin de garder des calculs simple on va convertir les secondes restante en milli secondes
  //Une seconde = 1000 mili secondes => Un extended square en longitude représentant 30s => un extended square en longitude représente 30000 millis secondes => un subsquare dans cet extended square en longitude représent 1250 milli secondes (30000/24)

  //Un extended square en latitude représentant 15s, un subsquare dans cet extended square en latitude représente 15/24 secondes , afin de garder des calculs simple on va convertir les secondes restante en milli secondes
  //Une seconde = 1000 mili secondes => Un extended square en latitude représentant 15s => un extended square en latitude représente 15000 millis secondes => un subsquare dans cet extended square en latitude représent 625 milli secondes (15000/24)

  lon5 = trunc(reste*1000/1250);
  reste = fmod(reste*1000,1250);


  //==============================CALCUL LATITUDE==================================

  //un carré en latitude = 10° (10*18 => 180°)
  //La latitude 0 GPS est décalé de 90° (20°*4.5) de la latitude "0" du MLS
  //Le premier caractère du MLS sera donc A si lat est dans l'intervale [-90;-80[
  //Il est necessaire de rajouter 90° = la valeur de lat pour être bien aligné

  lat+=90;

  la1=trunc(lat/10);
  reste = fmod(lat,10);

  la2=trunc(reste/1);
  reste = fmod(reste,1);

  la3 = trunc(reste*60/2.5);
  reste = fmod(reste*60,2.5);

  la4 = trunc(reste*60/15);
  reste = fmod(reste*60,15);

  la5 = trunc(reste*1000/625);
  reste = fmod(reste*1000,625);

  locator[0] = alphabet_maj[lon1];
  locator[1] = alphabet_maj[la1];
  locator[2] = (char) lon2 + 0x30;
  locator[3] = (char) la2 + 0x30;
  locator[4] = alphabet_maj[lon3];
  locator[5] = alphabet_maj[la3];
  locator[6] = (char) lon4 + 0x30;
  locator[7] = (char) la4 + 0x30;
  locator[8] = alphabet_min[lon5];
  locator[9] = alphabet_min[la5];
}

void getgps(TinyGPSPlus &gps)
{
  // Affichage --- latitude,longitude
  // --------------------------------
  oled.set1X();             // OLED 128x64
  oled.setCursor(0,0);
  oled.print(" Latitude:   ");
  oled.println(gps.location.lat(),5);
  oled.print(" Longitude:  ");  
  oled.println(gps.location.lng(),5);

  
  // Affichage --- Heure 
  // -------------------

  oled.setCursor(0,3);
  oled.print(" Heure: ");
      if (gps.time.hour() < 10)     // Add a leading zero
        oled.print("0");
      oled.print(gps.time.hour());  
  oled.print(":");
      if (gps.time.minute() < 10)   // Add a leading zero
        oled.print("0");
  oled.print(gps.time.minute());  
  oled.print(":");
      if (gps.time.second() < 10)   // Add a leading zero
        oled.print("0");
  oled.print(gps.time.second());
  oled.print("   ");

  
  // Affichage --- Sat + Locator
  // ---------------------------
  calcLocator(locator,gps.location.lat(),gps.location.lng());
  oled.setCursor(0,4); oled.print(" Sat:"); 
  if (gps.satellites.value() < 10)     // Add a leading zero
    oled.print("0");
  oled.print(gps.satellites.value());

  if (lock == 1)
    {
    oled.print("          Lock");
    }
  else
  {
    oled.print("        Unlock");
  }
  oled.setCursor(0,6);
  oled.print("  ");
  oled.set2X();
  oled.print (locator[0]);  
  oled.print (locator[1]); 
  oled.print (locator[2]); 
  oled.print (locator[3]); 
  oled.print (locator[4]); 
  oled.print (locator[5]); 
  oled.set1X();
  oled.print (locator[6]); 
  oled.print (locator[7]); 
  oled.print (locator[8]); 
  oled.println (locator[9]); 
  
}
  // 
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}

void calcChecksum(byte *checksumPayload, byte payloadSize) {
  byte CK_A = 0, CK_B = 0;
  for (int i = 0; i < payloadSize ;i++) {
    CK_A = CK_A + *checksumPayload;
    CK_B = CK_B + CK_A;
    checksumPayload++;
  }
  *checksumPayload = CK_A;
  checksumPayload++;
  *checksumPayload = CK_B;
}

void sendUBX(byte *UBXmsg, byte msgLength) {
  for(int i = 0; i < msgLength; i++) {
    gpsSerial.write(UBXmsg[i]);           // send UBX msg
    gpsSerial.flush();                    // Wait end of transmission
  }
    gpsSerial.println();              // send CR
    gpsSerial.flush();                // Wait end of transmission
}

byte getUBX_ACK(byte *msgID) {
  byte CK_A = 0, CK_B = 0;
  byte incoming_char;
  //boolean headerReceived = false;
  unsigned long ackWait = millis();
  byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int i = 0;
  while (1) {
    if (gpsSerial.available()) {
      incoming_char = gpsSerial.read();
      if (incoming_char == ackPacket[i]) { i++; }
      else if (i > 2) {ackPacket[i] = incoming_char;  i++; }
    }
    if (i > 9) break;
    if ((millis() - ackWait) > 1500) {
      Serial.println("ACK Timeout");
      return 5;
    }
    if (i == 4 && ackPacket[3] == 0x00) {
      Serial.println("NAK Received");
      return 1;
    }
  }

  for (i = 2; i < 8 ;i++) {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }
  
  if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9]) {
    Serial.println("Success!");
    Serial.println("ACK Received! ");
    return 10;
        }
  else {
    Serial.println("ACK Checksum Failure: ");
    delay(1000);
    return 1;
  }
}

void configureUblox() 
{
  byte gpsSetSuccess = 0;
 
    buf[0]=(frequency&0x000000FF);          // Preparation little endian
    buf[1]=(frequency&0x0000FF00)>>8;
    buf[2]=(frequency&0x00FF0000)>>16;
    buf[3]=(frequency&0xFF000000)>>24;

  //Generate the configuration string for TimePulse with frequency
  byte setTimePulse[] = {0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, buf[0], buf[1], buf[2], buf[3],buf[0], buf[1], buf[2], buf[3],0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,0x00,0xEF,0x00,0x00,0x00,0x91,0x22};
  calcChecksum(&setTimePulse[2], sizeof(setTimePulse) - 4);
  delay(1000);      // 1000

  while(gpsSetSuccess < 3)
  {
   Serial.println("Setting Timepulse... "); 
   gpsSetSuccess = 0;
   while (gpsSetSuccess < 3)
   {
   Serial.println("configuration TimePulse");
   sendUBX(&setTimePulse[0], sizeof(setTimePulse));
   gpsSetSuccess += getUBX_ACK(&setTimePulse[2]);         //Passes Class ID and Message ID to the ACK Receive function
   if ( (gpsSetSuccess == 10) ) gpsStatus[7] = true;
   if ( (gpsSetSuccess == 5) | (gpsSetSuccess == 6)) gpsSetSuccess -= 4;
   }
   if (gpsSetSuccess == 3){
    Serial.println("Timepulse non OK");
   }
  }
}

// --- Lecture et traitement de la tension de boucle et commande de la LED
void read_PLL()        
{
  int anal_loop;      // valeur sur 12bits lue sur l'entrée A0
  val_loop = 0;       // variable pour le traitement des valeurs courantes de la tension loop 

  old_moy_val_loop = moy_val_loop; 
  moy_val_loop = 0;

  for (int i=0; i<20; i++)        // boucle d'acquisition sur une durée de 200 ms 
    {
    anal_loop = analogRead(Port_anal_loop);
    val_loop = val_loop + anal_loop ;
    delay(10);
    }

  moy_val_loop = val_loop / 20 ;

  if ( ( moy_val_loop < (old_moy_val_loop - seuil_loop)) | ( moy_val_loop > (old_moy_val_loop + seuil_loop) )  )
    {
    digitalWrite(LED_loop, LOW);    // ----- turn Off the LED_loop  
    lock=0;
    }

  else
    {
    digitalWrite(LED_loop, HIGH);    // ----- turn On the LED_loop   
    lock=1;
    }
     

}
void setup()
{
  pinMode(LED_loop, OUTPUT);      // 
  digitalWrite(LED_loop, LOW);    // ----- turn Off the LED_loop  
  
  gpsSerial.begin(9600);    // GPS à 9600 Bauds
  Serial.begin(9600);       // Serial monitor  via USB at 9600 bauds ( au lieu de 115200 bauds )

  frequency=100000;         // 100 KHz  Timepulse Frequency in Hertz. The duty cycle is fixed at 50%
                            // only frequencies with even sub multiples numbers (ie, 2,4,6, 342 .....) are not noisy   for exemple :8MHz is ok, 10Khz is ok,
                            // 10 MHz is not ok
  configureUblox();         // Configure Timepulse

// Initialisation de l'afficheur I2C, OLED
// ----------------------------------------------
  Wire.begin();
  Wire.setClock(400000L);     // vitesse transfert i2c , 400000L ou 400.000 Hz

// Init OLED version 1
//********************
  #if RST_PIN >= 0                
    oled.begin(&SH1106_128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
    oled.begin(&SH1106_128x64, I2C_ADDRESS);
  #endif // RST_PIN >= 0


  oled.setFont(Adafruit5x7);

  //uint32_t m = micros();
  oled.clear();
  oled.set2X();
  oled.println("MultiGPSDO");
  oled.set1X();
  oled.println();
  oled.println("   40-25-10 MHz    ");
  oled.setCursor(0,3);
  oled.println("  Arduino Nano     ");
//  oled.println("     par F1TE      ");
  oled.set2X();
  oled.setCursor(0,6);
  oled.println(" par F6BUA ");
   
  delay(4000);  						// délai maintien de l'écran de démarrage pendant 4s

 // lcd.clear();
  oled.clear();

}

void loop()
{
smartDelay(1000);   // 1000 nearly 1.0s loop
getgps(gps);        // Get GPS parameters
read_PLL();         // Lecture et traitement de la tension de boucle
}