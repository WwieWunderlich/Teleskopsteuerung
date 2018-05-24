#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <Adafruit_MotorShield.h>
#include <stdlib.h>
#include <Wire.h>
#include <Keypad.h>
#include <MechaQMC5883.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *motor_ALT = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *motor_AZ = AFMS.getStepper(200, 2);
LiquidCrystal_I2C lcd(0x27, 20, 4);
TinyGPSPlus gps;
SoftwareSerial ss(11, 12);
const int MPU_addr = 0x68;
char datos[30];  //Daten von Serieller Schnittstelle
MechaQMC5883 qmc;
const byte COLS = 4; //3 Spalten
const byte ROWS = 4; //4 Zeilen

//Teleskop Koordinaten
char RA_telescopio[9];
char DEC_telescopio[10];
char RA_objetivo[9];
char DEC_objetivo[10];

char TIME_sideral[10];

double DEC_obj_dd, DEC_obj_mm, DEC_obj_ss, DEC_tel_dd, DEC_tel_mm, DEC_tel_ss;
double RA_obj_hh, RA_obj_mm, RA_obj_ss, RA_tel_hh, RA_tel_mm, RA_tel_ss;

double ALT_obj, AZ_obj, ALT_tel = 0, AZ_tel , LAT_dd, LAT_mm, LONG_dd, LONG_mm;
double HA_obj, A_obj, HA_tel, A_tel;
double TIME_hh, TIME_mm, TIME_ss, TIME_sid;

double DEC_obj_dez = (89.264166666666666666666666666667), RA_obj_dez  = (2.5302777777777777777777777777778), DEC_tel_dez, RA_tel_dez, LAT_beob_dez, LONG_beob_dez , TIME_dez;
int W_time_ALT, W_time_AZ, speed_M = 50;

char hexaKeys[ROWS][COLS] = {
  {'D', '#', '0', '*'},
  {'C', '9', '8', '7'},
  {'B', '6', '5', '4'},
  {'A', '3', '2', '1'}

};
byte colPins[COLS] = { 9, 8, 7, 6 }; //Definition der Pins für die 4 Spalten

byte rowPins[ROWS] = { 5, 4, 3, 2 };//Definition der Pins für die 4 Zeilen

char P_Key; //pressedKey entspricht in Zukunft den gedrückten Tasten

Keypad M_Keypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); //Das Keypad kann absofort mit myKeypad angesprochen werden

//Setup
void setup() {
  Serial.begin(9600);
  ss.begin(9600);
  Serial.println("Teleskopsterung TW");
  motor_ALT->setSpeed(speed_M);
  motor_AZ->setSpeed(speed_M);
  AFMS.begin();
  lcd.init();                      // initialisiere das LCD
  lcd.backlight();
  qmc.init();
  Wire.begin();
  setSyncProvider(RTC.get);
  /*LAT_dd = double(50);
    LAT_mm = double(59.2);
    LONG_dd = double(12);
    LONG_mm = double(26); */
  /*RA_obj_dez = double(5.278333);
    RA_tel_dez = double(5.278333);
    DEC_obj_dez = double(45.99575);
    DEC_tel_dez = double(45.99575);*/


  Menu();
  BACK_Koor_Tran();
}
#define DRAW_DELAY 118
#define D_NUM 47

void loop() {
  if (Serial.available() > 0) {
    leer_datos_serie();
  }
  BACK_Koor_Tran();
  Koor_Tran();
  Koor_Zusammen();

}

void gro_alt() {
  double a = ALT_obj - ALT_tel;
  ALT_tel = ALT_tel + double(a); //Übersetzungsverhältnis 1:15 bei 200 Schritt
  motor_ALT->step(int(a / 0.12), BACKWARD, SINGLE);
  W_time_ALT = ((60 * a) / (speed_M * 3000));
}

void kleiner_alt() {
  double a = ALT_tel - ALT_obj;
  ALT_tel = ALT_tel - double(a) ;
  motor_ALT->step(int(a / 0.12), FORWARD, SINGLE);
  W_time_ALT = ((60 * a) / (speed_M * 3000));
}

void gro_az() {
  double a = AZ_obj - AZ_tel;
  AZ_tel = AZ_tel + double(0.12);
  motor_AZ->step(int(a / 0.12), FORWARD, SINGLE );
  W_time_AZ = ((60 * a) / (speed_M * 3000));
}

void kleiner_az() {
  double a = AZ_tel - AZ_obj;
  AZ_tel = AZ_tel - double(a);
  motor_AZ->step(int(a / 0.12), BACKWARD, SINGLE);
  W_time_AZ = ((60 * a) / (speed_M * 3000));
}

void Koor_Zusammen() {
  if (ALT_tel > ALT_obj) {
    kleiner_alt();
  }
  if (ALT_tel < ALT_obj) {
    gro_alt();
  }
  if (AZ_tel > AZ_obj) {
    kleiner_az();
  }
  if (AZ_tel < AZ_obj) {
    gro_az();
  }
  delay(max(W_time_ALT, W_time_AZ) * 1000);
}

//Daten Empfang aus serieller Schnittstelle (USB)
void leer_datos_serie() {
  int i = 0;
  datos[i++] = Serial.read();
  delay(5);
  while ((datos[i++] = Serial.read()) != '#') { //LX-200 Protokoll sendete #
    delay(5);                                   //5ms delay
  }
  datos[i] = '\0';
  auswertung_daten();                              //Auswertung der Daten
}

void Data_keypad() {
  byte data_count = 0;
  char Data[10];
  while ((P_Key = M_Keypad.getKey()) != '#') {

    if (P_Key)
    {

      Data[data_count] = P_Key;
      lcd.setCursor(3, 0);
      lcd.print(Data);
      data_count++;
    }
  }
}

void Menu() {
  char Data[10];
  byte data_count = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Auto. Eingabe A");
  lcd.setCursor(0, 2);
  lcd.print("Manuelle Eingabe B");
   while ((P_Key = M_Keypad.getKey()) != '#') {

    if (P_Key)
    {

      Data[data_count] = P_Key;
      lcd.setCursor(5, 3);
      lcd.print(Data);
      data_count++;
    }
  }
  switch (Data[0]) {
    case ('A'):
      lcd.clear();
      lcd.setCursor (0, 0);
      lcd.print("Automatische Eingabe");
      Setup_daten_automatisch();
      break;
    case ('B'):
      lcd.clear();
      lcd.setCursor(0, 2);
      lcd.print("Manuelle Eingabe");
      Setup_daten_manuel();
      break;
  }
}
int Z=5*1000;
void Setup_daten_automatisch() {
  GPS_data();
  AZ_data();
  ALT_data();
}
  
void GPS_data() {
  lcd.setCursor(1,0);
  lcd.print("GPS Fix wird veranlasst");
  delay(Z/2);
  lcd.clear();
  int s = 0, d = 0;
 while (s < 1 && d < 1){  
  while (ss.available() > 0)
    gps.encode(ss.read());
    
     if (gps.location.isValid()) {
    LAT_beob_dez = double(gps.location.lat());
    LONG_beob_dez = double(gps.location.lng());
    ++s;

  }
  if (gps.date.isValid() || gps.time.isValid()) {
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
    ++d;
        }
      
 }
 lcd.setCursor(0, 0);
    lcd.print("Geo Daten");
    lcd.setCursor(0, 1);
    lcd.print(int(LAT_beob_dez*10)/10);
    lcd.setCursor(4,1);
    lcd.print(int(LONG_beob_dez*10)/10);
    lcd.setCursor(0, 2);
    lcd.print("Zeit Daten");
    lcd.setCursor(0, 3);
    lcd.print(hour());
    lcd.setCursor(2, 3);
    lcd.print(':');
    lcd.setCursor(3, 3);
    lcd.print(minute());
    lcd.setCursor(4, 3);
    lcd.print(':');
    lcd.setCursor(6, 3);
    lcd.print(second());
    delay(Z);
}
  
void AZ_data() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Mangetometer wird ausgewertet");
  delay(Z/2);
  int x, y, z;
  int azimuth;
  double a;
  qmc.read(&x, &y, &z,&azimuth);
  a = azimuth;
  double azimuth_real = (3.0 + (31.0 / 60.0)) / 360;
  a += double(azimuth_real);
  if (a < 0)
  {
    a += 360;
  }

  if (a > 360)
  {
    a -= 360;
  }

  AZ_tel = double(a);
  lcd.clear();
  lcd.setCursor(1,1);
  lcd.print(AZ_tel);
  delay(Z);
}
 void ALT_data() {
 
  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print("Accelometer wird ausgewertet");
  delay(Z/2);
   int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

  int minVal = 265;
  int maxVal = 402;
  int xAng;
  int yAng;
  int zAng;
  double x;
  double y;
 double z;
 
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  xAng = map(AcX, minVal, maxVal, -90, 90);
  yAng = map(AcY, minVal, maxVal, -90, 90);
  zAng = map(AcZ, minVal, maxVal, -90, 90);
  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);
  ALT_tel = double(x);
 
  lcd.setCursor(2,2);
  lcd.print(x);
  lcd.setCursor(2,5);
  lcd.print(y);
  lcd.setCursor(2,7);
  lcd.print(z);
  lcd.setCursor(3,1);
  lcd.print(xAng);
  lcd.setCursor(3,4);
  lcd.print(yAng);
  lcd.setCursor(3,6);
  lcd.print(zAng);
  lcd.setCursor (0,2);
  lcd.print(ALT_tel);
  delay(Z);
} 
void Setup_daten_manuel () {
  byte data_count = 0;
  char Data[10];
  lcd.init();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Geographische Breite");
  lcd.setCursor(2, 2);
  lcd.print("'");
  P_Key = M_Keypad.getKey();
  while ((P_Key = M_Keypad.getKey()) != '#')
  {

    if (P_Key)
    {

      Data[data_count] = P_Key;
      lcd.setCursor(data_count, 2);
      lcd.print (Data[data_count]);
      lcd.setCursor(0, 3);
      lcd.print(data_count);
      data_count++;
    }
    if ( data_count == 2) {
      data_count = 3;
    }
  }
  LAT_beob_dez = ((((Data[0] - 48) * 10) + (Data[1] - 48)) + ((((Data[3] - 48) * 10) + (Data[4] - 48)) / double(60)));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(LAT_beob_dez);
  delay(2000);

  data_count = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Geographische Laenge");
  lcd.setCursor(2, 2);
  lcd.print("'");
  P_Key = M_Keypad.getKey();
  while ((P_Key = M_Keypad.getKey()) != '#')
  {

    if (P_Key)
    {

      Data[data_count] = P_Key;
      lcd.setCursor(data_count, 2);
      lcd.print (Data[data_count]);
      lcd.setCursor(0, 3);
      lcd.print(data_count);
      data_count++;
    }
    if ( data_count == 2) {
      data_count = 3;
    }
  }
  LONG_beob_dez = ((((Data[0] - 48) * 10) + (Data[1] - 48)) + ((((Data[3] - 48) * 10) + (Data[4] - 48)) / double(60)));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(LONG_beob_dez);
  delay(2000);

  data_count = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Auswahl Objekt:");
  lcd.setCursor(0, 1);
  lcd.print("A Andromeda");
  lcd.setCursor(0, 2);
  lcd.print("B Jupiter");
  lcd.setCursor(0, 3);
  lcd.print("C Capella");
  lcd.setCursor(10, 3);
  lcd.print("D Eingabe");

  Data_keypad();
  lcd.clear();
  lcd.setCursor(0, 0);
  switch (Data[0]) {
    case ('A'):
      RA_obj_dez = double(0.71231388889);
      DEC_obj_dez = double(41.26875);
      lcd.print("Andromeda");
      break;
    case ('B'):
      RA_obj_dez = 15.37648889;
      DEC_obj_dez = 17.3208889;
      break;
      lcd.print("Jupiter");
    case ('C'):
      RA_obj_dez = 5.2782222;
      DEC_obj_dez = 45.99575;
      lcd.print("Capella");
      break;
    case ('D'):
      data_count = 0;
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("Eingabe:");
      lcd.setCursor(0, 1);
      lcd.print("Rektaszension:");
      lcd.setCursor(8, 3);
      lcd.print("Deklination:");
      lcd.setCursor(2, 2);
      lcd.print("'");
      lcd.setCursor(5, 2);
      lcd.print("'");
      lcd.setCursor(12, 2);
      lcd.print("'");
      lcd.setCursor(15, 2);
      lcd.print("'");
      P_Key = M_Keypad.getKey();
      while ((P_Key = M_Keypad.getKey()) != '#')
      {

        if (P_Key)
        {

          Data[data_count] = P_Key;
          lcd.setCursor(data_count, 2);
          lcd.print (Data[data_count]);
          lcd.setCursor(0, 3);
          lcd.print(data_count);
          data_count++;
        }
        if ( data_count == 2 || data_count == 5 || data_count == 12 || data_count == 15) {
          data_count++;
        }
        if (data_count == 8) {
          data_count = 10;
        }

      }
      RA_obj_dez = (((Data[0] - 48) * 10) + (Data[1] - 48)) + ((((Data[3] - 48) * 10) + (Data[4] - 48)) / double(60));
      DEC_obj_dez = (((Data[10] - 48) * 10) + (Data[11] - 48)) + ((((Data[13] - 48) * 10) + (Data[14] - 48)) / double(60));
      delay(1000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Auswahl erfolgreich!");
      delay(3000);
      lcd.clear();
      break;
  }
  lcd.setCursor(0, 0);
  lcd.print("Setup wird beendet und Verfolgung gestartet!");
  delay(3000);
  lcd.clear();

}

void Koor_Tran() {
  /* long DEC_obj_dd, DEC_obj_mm, DEC_obj_ss,DEC_tel_dd, DEC_tel_mm, DEC_tel_ss;
    long RA_obj_hh, RA_obj_mm, RA_obj_ss, RA_tel_hh, RA_tel_mm, RA_tel_ss;

    long RA_tel, DEC_tel, RA_obj, DEC_obj, LAT_beob;
    double ALT_obj, AZ_obj,ALT_tel, AZ_tel, LAT_dd = double(50), LAT_mm = double(59.2);

    long TIME_hh, TIME_mm, TIME_ss, TIME_sid;

    double DEC_obj_dez, RA_obj_dez, DEC_tel_dez, RA_tel_dez, LAT_beob_dez,TIME_dez;
  */


  Sternzeit();
  HA_tel = TIME_dez - (RA_tel_dez * double(15)) ;
  while (HA_tel < 0 || HA_tel > 360) {
    if (HA_tel < 0) {
      HA_tel = HA_tel + double(360);
    }
    if (HA_tel > 360) {
      HA_tel = HA_tel - double(360);
    }
  }
  ALT_tel = degrees(asin(sin(radians(DEC_tel_dez)) * sin(radians(LAT_beob_dez)) + (cos(radians(DEC_tel_dez)) * cos(radians(LAT_beob_dez)) * cos(radians(HA_tel)))));
  AZ_tel = degrees(atan((sin(radians(HA_tel))) / ((cos(radians(HA_tel)) * sin(radians(LAT_beob_dez))) - (cos(radians(LAT_beob_dez)) * tan(radians(DEC_tel_dez))))));

  Sternzeit();
  HA_obj = TIME_dez - (RA_obj_dez * double(15)) ;
  while (HA_obj  < 0 || HA_obj > 360) {
    if (HA_obj < 0) {
      HA_obj = HA_obj + double(360);
    }
    if (HA_obj > 360) {
      HA_obj = HA_obj - double(360);
    }
  }
  ALT_obj = degrees(asin(sin(radians(DEC_obj_dez)) * sin(radians(LAT_beob_dez)) + (cos(radians(DEC_obj_dez)) * cos(radians(LAT_beob_dez)) * cos(radians(HA_obj)))));
  AZ_obj = degrees(atan( (sin(radians(HA_obj))) / ( ( cos(radians(HA_obj)) * sin (radians(LAT_beob_dez)) ) - ( cos(radians(LAT_beob_dez)) * tan(radians(DEC_obj_dez)) ) )));

  while (AZ_tel  < 0 || AZ_tel > 360) {
    if (AZ_tel < 0) {
      AZ_tel = AZ_tel + double(360);
    }
    if (HA_obj > 360) {
      AZ_tel = AZ_tel - double(360);
    }
  }

  while (AZ_obj  < 0 || AZ_obj > 360) {
    if (AZ_obj < 0) {
      AZ_obj = AZ_obj + double(360);
    }
    if (HA_obj > 360) {
      AZ_obj = AZ_obj - double(360);
    }
  }




  lcd.setCursor(10, 0);
  lcd.print(ALT_obj);
  lcd.setCursor(10, 1);
  lcd.print(AZ_obj);
  lcd.setCursor(0, 0);
  lcd.print(ALT_tel);
  lcd.setCursor(0, 1);
  lcd.print(AZ_tel);
}

void Sternzeit() {
  double d_month, d_year, d_gesamt, TIME_z;
  int i_360;
  switch (month()) {
    case (1):
      d_month = 0;
      break;
    case (2):
      d_month = 31;
      break;
    case (3):
      d_month = 59;
      break;
    case (4):
      d_month = 90;
      break;
    case (5):
      d_month = 120;
      break;
    case (6):
      d_month = 151;
      break;
    case (7):
      d_month = 181;
      break;
    case (8):
      d_month = 212;
      break;
    case (9):
      d_month = 243;
      break;
    case (10):
      d_month = 273;
      break;
    case (11):
      d_month = 304;
      break;
    case (12):
      d_month = 334;
      break;
  }
  switch (year()) {
    case (2005):
      d_year = 1825.5;
      break;
    case (2006):
      d_year = 2190.5 ;
      break;
    case (2007):
      d_year = 2555.5;
      break;
    case (2008):
      d_year = 2920.5;
      break;
    case (2009):
      d_year = 3286.5;
      break;
    case (2010):
      d_year = 3651.5;
      break;
    case (2011):
      d_year = 4016.5;
      break;
    case (2012):
      d_year = 4381.5;
      break;
    case (2013):
      d_year = 4747.5;
      break;
    case (2014):
      d_year = 5112.5 ;
      break;
    case (2015):
      d_year = 5477.5;
      break;
    case (2016):
      d_year = 5842.5;
      break;
    case (2017):
      d_year = 6208.5;
      break;
    case (2018):
      d_year = 6573.5;
      break;
    case (2019):
      d_year = 6938.5;
      break;
    case (2020):
      d_year = 7303.5;
      break;
    case (2021):
      d_year = 7669.5;
      break;
  }
  d_gesamt = (((second() / double(3600)) + (minute() / double(60)) + hour()) / double(24)) + d_month + d_year + day();
  TIME_z = 100.46 + (double(0.985647) * d_gesamt) + LONG_beob_dez + ((hour() + (minute() / double(60)) + (second() / double(3600))) * double(15));
  TIME_dez = TIME_z;
  while (TIME_dez > 360 || TIME_dez < 0) {
    if (TIME_dez > 360) {
      TIME_dez = TIME_dez - 360;
    }
    if (TIME_dez < 0) {
      TIME_dez = TIME_dez + 360;
    }

  }

}
void BACK_Koor_Tran() {
  /*long DEC_obj_dd, DEC_obj_mm, DEC_obj_ss,DEC_tel_dd, DEC_tel_mm, DEC_tel_ss;
    long RA_obj_hh, RA_obj_mm, RA_obj_ss, RA_tel_hh, RA_tel_mm, RA_tel_ss;

    long RA_tel, DEC_tel, RA_obj, DEC_obj, LAT_beob;
    double ALT_obj, AZ_obj,ALT_tel, AZ_tel, LAT_dd = double(50), LAT_mm = double(59.2);

    long TIME_hh, TIME_mm, TIME_ss, TIME_sid;

    double DEC_obj_dez, RA_obj_dez, DEC_tel_dez, RA_tel_dez, LAT_beob_dez,TIME_dez;
  */
  Sternzeit();

  DEC_tel_dez = degrees(asin(sin(radians(LAT_beob_dez)) * sin (radians(ALT_tel)) - cos (radians(LAT_beob_dez)) * cos (radians(ALT_tel)) * cos (radians(AZ_tel))));
  RA_tel_dez = (TIME_dez - degrees(atan( (sin(radians(AZ_tel))) / ((cos(radians(AZ_tel)) * sin(radians(LAT_beob_dez))) + cos(radians(LAT_beob_dez)) * tan(radians(ALT_tel)))))) / double(15);



  DEC_tel_dd = int(DEC_tel_dez);
  DEC_tel_mm = (DEC_tel_dez - DEC_tel_dd) * 60;
  DEC_tel_ss = ((DEC_tel_mm - int(DEC_tel_mm)) * 60) ;
  RA_tel_hh = int(RA_tel_dez);
  RA_tel_mm = ((RA_tel_dez) - RA_tel_hh) * 60;
  RA_tel_ss = (RA_tel_mm - int(RA_tel_mm)) * 60;

  lcd.setCursor(10, 2);
  lcd.print(DEC_obj_dez);
  lcd.setCursor(10, 3);
  lcd.print(RA_obj_dez);
  lcd.setCursor(0, 2);
  lcd.print(DEC_tel_dez);
  lcd.setCursor(0, 3);
  lcd.print(RA_tel_dez);


  /*lcd.clearlcd();
    lcd.setTextColor(WHITE);
    lcd.setCursor(50,0);
    lcd.setTextSize(1);
    lcd.println(ALT_tel);
    lcd.setCursor(50,10);
    lcd.println(ALT_obj);
    lcd.setCursor(50,20);
    lcd.println(DEC_tel_dez);
    lcd.lcd();*/
}

//LX-200 Protokoll Daten
void auswertung_daten() {

  // #:GR#  -> RA vom Teleskop
  if (datos[1] == ':' && datos[2] == 'G' && datos[3] == 'R' && datos[4] == '#') {
    sprintf(RA_telescopio, "%02d:%02d:%02d", int(RA_tel_hh), int(RA_tel_mm), int(RA_tel_ss));
    Serial.print(RA_telescopio);
    Serial.print("#");
  }

  // #:GD#  -> DEC vom Teleskop
  if (datos[1] == ':' && datos[2] == 'G' && datos[3] == 'D' && datos[4] == '#') {
    sprintf(DEC_telescopio, "%+03d:%02d:%02d", int(DEC_tel_dd), int(DEC_tel_mm), int(DEC_tel_ss));
    Serial.print(DEC_telescopio);
    Serial.print("#");


  }

  // #:Q#
  if (datos[1] == ':' && datos[2] == 'Q' && datos[3] == '#') {
  }                                                                        // <<<---- Zukünftige Nutzung vorhgesehen

  // :Sr HH:MM:SS#  -> RA vom Objekt
  if (datos[0] == ':' && datos[1] == 'S' && datos[2] == 'r') {
    for (int i = 0; i < 8; i++)
      RA_objetivo[i] = datos[i + 4];
    Serial.print("1");

    RA_obj_hh = atoi(datos + 4);
    RA_obj_mm = atoi(datos + 7);
    RA_obj_ss = atoi(datos + 10);
    RA_obj_dez = double( RA_obj_hh + (RA_obj_mm / double(60)) + (RA_obj_ss / double(3600)));

  }

  // :Sd sDD*MM:SS# ->DEC vom Objekt
  if (datos[0] == ':' && datos[1] == 'S' && datos[2] == 'd') {
    for (int i = 0; i < 9; i++)
      DEC_objetivo[i] = datos[i + 4];
    Serial.print("1");

    DEC_obj_dd = atoi(datos + 4); //148560 -> 41 16
    DEC_obj_mm = atoi(datos + 8);
    DEC_obj_ss = atoi(datos + 11);
    DEC_obj_dez = double( DEC_obj_dd + (DEC_obj_mm / double(60)) + (DEC_obj_ss / double(3600)));
  }

  // :MS# -> Schwenken möglich?
  if (datos[0] == ':' && datos[1] == 'M' && datos[2] == 'S' && datos[3] == '#') {
    Serial.print("0");                                                     // <<<---- Schwenken ist Möglich 0
  }

}
// UTC Zeit wird über RTC Modul abgefragt werden; hier Test über Serial


