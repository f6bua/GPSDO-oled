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

void getgps(TinyGPSPlus &gps);

float latitude;
float longitude;
char locator[6];



// Calcul du locator
void calcLocator(char *dst,float lat, float lng )     // Calcule le Locator ...
{
  int lon1, lon2, lon3, lon4, lon5;
  int la1, la2, la3, la4, la5;
  float reste;
  
  // longitude  
  reste = lng + (20*9);                     // On ajoute le décalage entre le 0 du locator et le 0 de Longitude
   // Paquets de 20° 
  lon1 = int(reste / 20.0);                 //
  reste = reste - float(lon1) * 20.0;       //
  // Paquets de 20/10 = 2°
  lon2 = int(reste / 2.0);                  //
  reste = reste - float(lon2) * 2;          //
  // Paquets de 2°/24 = 5'
  lon3 = int(reste / (5/60));               //
  reste = reste - float(lon3)*(5/60);       //
  // Paquets de 5'/10 = 30"
  lon4 = int(reste /(30/3600));             //
  reste = reste - float(lon4)*(30/3600);    //
  // Paquets de 30"/10 = 3"
  lon5 = int(reste/(3/3600));               //
  

  // latitude
  reste = lat + 90.0;                       // On ajoute 90° car le locator commence au pole Sud
  // Paquets de 10°
  la1 = int(reste / 10.0);                  // 
  reste = reste - float(la1) * 10.0;        // 
  // Paquets de 10/10 = 1°
  la2 = int(reste / 1);                     // 
  reste = reste - float(la2) * 1;           //
  // Paquets de 1°/24 = 2,5'
  la3 = int(reste / (2,5/60));              // 
  reste = reste - float(la3)*(2,5/60);      // 
  // Paquets de 2,5'/10 = 15"
  la4 = int(reste / (15/3600));             //
  reste = reste - float(la4)*(15/3600);     // 
    // Paquets de 15"/10 = 1,5"
  la5 = int(reste / (1.5/3600));            //
  
  dst[0] = (char)lon1 + 65;     //  9 + 65 = 74 --> J
  dst[1] = (char)la1 + 65;      // 14 + 65 = 79 --> O
  dst[2] = (char)lon2 + 48;     // 1 + 48 = 49 --> 1
  dst[3] = (char)la2 + 48;      // 0 + 48 = 48 --> 0
  dst[4] = (char)lon3 + 65;     // 18 + 65 = 83 --> S
  dst[5] = (char)la3 + 65;      //  8 + 65 = 74 --> I
  dst[6] = (char)lon4 + 48;     // 0 + 48 = 48 --> 0
  dst[7] = (char)la4 + 48;      // 6 + 48 = 54 --> 6
  dst[8] = (char)lon5 + 65;     // 23 + 65 = 88 --> X
  dst[9] = (char)la5 + 65;      // 17 + 65 = 82 --> R     
}

void getgps(TinyGPSPlus &gps)
{
  // Affichage --- latitude,longitude
  // --------------------------------
  oled.set1X();             // OLED 128x64
  oled.setCursor(0,0);
  oled.print(" Lat   ");
  oled.println(gps.location.lat(),5);
  oled.print(" Long  ");  
  oled.println(gps.location.lng(),5);

  
  // Affichage --- Heure 
  // -------------------

  oled.setCursor(0,3);
  oled.print(" Heure ");
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
  oled.setCursor(0,4); oled.print(" Sat   "); 
  if (gps.satellites.value() < 10)     // Add a leading zero
    oled.print("0");
  oled.print(gps.satellites.value());

  if (lock == 1)
    {
    oled.print("  Lock   ");
    }
  else
  {
   oled.print("  Unlock");
  }
  oled.setCursor(0,6);
  oled.print("   ");
  oled.set2X();
  oled.print (locator[0]);  
  oled.print (locator[1]); 
  oled.print (locator[2]); 
  oled.print (locator[3]); 
  oled.print (locator[4]); 
  oled.println (locator[5]); 
  oled.set1X();
  
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
  oled.println("    Multi  GPSDO   ");
  oled.set1X();
  oled.println("    40-25-10 MHz   ");
  oled.setCursor(0,3);
  oled.println("    Arduino NEO-8M ");
  oled.setCursor(0,5);
  oled.println(" de F1TE mod F6BUA ");
   
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