#include <Arduino.h>
#include <math.h> // L'opérateur modulo (%) ne fonctionne qu'avec des entiers, pour les flotants la lib math contient la fonction fmod()
 
 float fixed_lat = 50.3614037;//50.3105962;
 float fixed_lon = 3.5081121; //3.4885149;

 char locator[11] = {'_','_','_','_','_','_','_','_','_','_',0};
 
 //Ressources : 
 //https://en.wikipedia.org/wiki/Maidenhead_Locator_System
 //https://en.wikipedia.org/wiki/Maidenhead_Locator_System#/media/File:Maidenhead_Locator_Map.png
 //http://n1sv.com/PROJECTS/PROJECTS.htm

 //IMPORTANT : 
 // - Les 2 premiers caractères vont de A à R (base 18) =>  JO (Field)
 // - Les 2 suivants de 0 à 9 (base 10) => 10 (square)
 // - Les 2 suivants de a à x (base 24) => rh (subsquare)
 // - Les 2 suivants de 0 à 9 (base 10) => 84 (extended square)

 // - Les caractères suivant ne sont pas formellement défini dans le MLS
 // - Toutefois on peut les calculer en utilisant une alternance de "subsquare" (base24) et de "extended square" (base10)

// Calcul du locator
void calcLocator(char *dst,float lat, float lng )     // Calcule le Locator ...
{
  int lon1, lon2, lon3, lon4, lon5;
  int la1, la2, la3, la4, la5;
  float reste;

  //Les 2 lignes suivantes peuvent être remplacer par des boucles for pour initialiser le tableau
  char alphabet_maj[26]={'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'};
  char alphabet_min[26]={'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z'};
  

 //==============================CALCUL LONGITUDE==================================

  //Un carré en longitude = 20° (20*18 => 360°)
  //La longitude 0 GPS est décallé de 180° (20°*9) de la longitude "0" du MLS
  //Le premier caractère du MLS sera donc A si long est dans l'intervale [-180;-160[
  //Il est necessaire de rajouter 180° = la valeur de lng pour être bien aligné
  lng+=180;

  Serial.println("Longitude (+180deg)");
  Serial.println(lng);

  //Calcul lon1
  Serial.println("Calcul lon1");
  lon1=trunc(lng/20);
  reste = fmod(lng,20);
  Serial.println("Valeur lon1");
  Serial.println(lon1, 6);
  Serial.println("Reste lon1");
  Serial.println (reste, 6);

  //Une fois le "Field" calculé, il faut maintenant calculter la position du square dans ce field
  //Dans un field il y a 100 square (une "matrice" de 10X10)
  //Un field en longitude représentant 20°, un square en longitude représente 2° (20/10)
  //Un field en lattitude représentant 10°, un square en lattitude représente 1° (10/10)

  Serial.println("Calcul lon2");
  lon2 = trunc(reste/2);
  reste = fmod(reste,2);
  Serial.println("Valeur lon2");
  Serial.println(lon2, 6);
  Serial.println("Reste lon2");
  Serial.println (reste, 6);

  //Une fois le "square" calculé, il faut maintenant calculter la position du subsquare dans ce square
  //Dans un square il y a 576 subsquare (une "matrice" de 24*24)
  //Un square en longitude représentant 2°, un subsquare en longitude représente 2/24 ° , afin de garder des calculs simple on va convertir les degrés restant en minute
  //Un degrés = 60 minutes => un square en longitude représente 2° => un square en longitude représente 120minutes => un subsquare en longitude représent 5minutes (120/24)

  //Un square en latitude représentant 1°, un subsquare en latitude représente 1/24 ° , afin de garder des calculs simple on va convertir les degrés restant en minute
  //Un degrés = 60 minutes => un square en latitude représente 1° => un square en latitude représente 60minutes => un subsquare en latitude représent 2.5 minutes (60/24)

  Serial.println("Calcul lon3");
  lon3 = trunc(reste*60/5);
  reste = fmod(reste*60,5);
  Serial.println("Valeur lon3");
  Serial.println(lon3, 6);
  Serial.println("Reste lon3");
  Serial.println (reste, 6);

  //Une fois le "subsquare" calculé, il faut maintenant calculter la position de l'extended square dans ce subsquare
  //Dans un subsquare il y a 100 extend quare (une "matrice" de 10)

  //Un subsquare en longitude représentant 5minutes, un extended square en longitude représente 5/10 minute , afin de garder des calculs simple on va convertir les minutes restante en secondes
  //Une minutes = 60 secondes => un subsquare en longitude représente 5 minutes => un subsquare en longitude représente 300 secondes => un extended square en longitude représent 30s (300/10)

  //Un subsquare en latitude représentant 1°, un extended quare en latitude représente 1/10 minute , afin de garder des calculs simple on va convertir les minutes restantes en secondes
  //Une minute = 60 secondes => un square en latitude représente 2.5 minutes => un square en latitude représente 150 minutes => un extended square en latitude représent 15 minutes (150/10)

  Serial.println("Calcul lon4");
  lon4 = trunc(reste*60/30);
  reste = fmod(reste*60,30);
  Serial.println("Valeur lon4");
  Serial.println(lon4, 6);
  Serial.println("Reste lon4");
  Serial.println (reste, 6);

  //Une fois l extended square calculé, on peut calculer un subsquare dans cet extended subsquare
  //Dans un extended square il y a 576 subsquare (une "matrice" de 24X24)

  //Un extended square en longitude représentant 30s, un subsquare dans cet extended square en longitude représente 30/24 secondes , afin de garder des calculs simple on va convertir les secondes restante en milli secondes
  //Une seconde = 1000 mili secondes => Un extended square en longitude représentant 30s => un extended square en longitude représente 30000 millis secondes => un subsquare dans cet extended square en longitude représent 1250 milli secondes (30000/24)

  //Un extended square en latitude représentant 15s, un subsquare dans cet extended square en latitude représente 15/24 secondes , afin de garder des calculs simple on va convertir les secondes restante en milli secondes
  //Une seconde = 1000 mili secondes => Un extended square en latitude représentant 15s => un extended square en latitude représente 15000 millis secondes => un subsquare dans cet extended square en latitude représent 625 milli secondes (15000/24)

  Serial.println("Calcul lon5");
  lon5 = trunc(reste*1000/1250);
  reste = fmod(reste*1000,1250);
  Serial.println("Valeur lon5");
  Serial.println(lon5, 6);
  Serial.println("Reste lon5");
  Serial.println (reste, 6);




  //==============================CALCUL LATTITUDE==================================

  //un carré en lattitude = 10° (10*18 => 180°)
  //La lattitude 0 GPS est décallé de 90° (20°*4.5) de la lattitude "0" du MLS
  //Le premier caractère du MLS sera donc A si lat est dans l'intervale [-90;-80[
  //Il est necessaire de rajouter 90° = la valeur de lat pour être bien aligné

  lat+=90;
  Serial.println("Lattitude (+90 deg)");
  Serial.println(lat);

  Serial.println("Calcul la1");
  la1=trunc(lat/10);
  reste = fmod(lat,10);
  Serial.println(la1, 6);
  Serial.println(reste, 6);

  Serial.println("Calcul la2");
  la2=trunc(reste/1);
  reste = fmod(reste,1);
  Serial.println(la2, 6);
  Serial.println(reste, 6);

  Serial.println("Calcul la3");
  la3 = trunc(reste*60/2.5);
  reste = fmod(reste*60,2.5);
  Serial.println("Valeur la3");
  Serial.println(la3, 6);
  Serial.println("Reste la3");
  Serial.println (reste, 6);

  Serial.println("Calcul la4");
  la4 = trunc(reste*60/15);
  reste = fmod(reste*60,15);
  Serial.println("Valeur la4");
  Serial.println(la4, 6);
  Serial.println("Reste la4");
  Serial.println (reste, 6);

  Serial.println("Calcul la5");
  la5 = trunc(reste*1000/625);
  reste = fmod(reste*1000,625);
  Serial.println("Valeur la5");
  Serial.println(la5, 6);
  Serial.println("Reste la5");
  Serial.println (reste, 6);

  locator[0] = alphabet_maj[lon1];
  locator[1] = alphabet_maj[la1];
  locator[2] = (char) lon2 + 0x30;
  locator[3] = (char) la2 + 0x30;
  locator[4] = alphabet_min[lon3];
  locator[5] = alphabet_min[la3];
  locator[6] = (char) lon4 + 0x30;
  locator[7] = (char) la4 + 0x30;
  locator[8] = alphabet_min[lon5];
  locator[9] = alphabet_min[la5];
}


void setup()
{
 Serial.begin(9600);
 calcLocator(locator, fixed_lat, fixed_lon);
 Serial.println("Locator : ");
 Serial.println(locator);
}

void loop()
{
  delay(10000);
}