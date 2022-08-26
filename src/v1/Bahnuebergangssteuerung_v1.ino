// #include <Keyboard.h>

#include <EEPROM.h>

#include <avr/pgmspace.h> // Wird zum Ablegen von Werten im FLASH benoetig

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <Button.h>

#include <SPI.h>
#include <SD.h>                   
#include <TMRpcm.h>

//#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINT(x)     Serial.print (x)
 #define DEBUG_PRINTDEC(x)     Serial.print (x, DEC)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTLN(x) 
#endif

/* Bahnuebergangsteuerung v0.12

Treiber fuer den PWM-Chip von Adafruit https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/archive/master.zip

Debouncing der Tasten ueber Button(carlynorma) Bibliothek https://github.com/carlynorama/Arduino-Library-Button/

Audiowiedergabe ber TMRpcm Bibliothek https://github.com/TMRh20/TMRpcm

Belegung der SD-Karte:
MOSI - pin 11, MISO - pin 12, CLK - pin 13, CS - pin 10
Verwendet fuer Leonardo angepasste Version der SD Bibliothek von Adafruit https://github.com/adafruit/SD/archive/master.zip

*/

// MR

// CRC32 (Christopher Baker)
#include <CRC32.h>

struct SchrankenDaten {
   int PosUnten;        // Position der Stellung "Unten"
   int PosOben;         // Position der Stellung "Oben"  
   int updateInterval;   // wie schnell wird die Schranke bewegt
};

struct EEPROMPayload {
  SchrankenDaten schranke[8];  // in dieser Konfiguration maximal acht Schranken
};

struct EEPROMData {
  EEPROMPayload payload;
  uint32_t crc;
};

EEPROMData eepromData;

// /MR


// Ausgangspins
#define blink1_OUT 8    // hier wird das erste Blinklicht angeschlossen
#define blink2_OUT 9    // hier wird das zweite Blinklicht angeschlossen
#define blink3_OUT 10   // hier wird das dritte Blinklicht angeschlossen
#define blink4_OUT 11   // hier wird das vierte Blinklicht angeschlossen
#define uebsig1_OUT 12  // hier wird das rechte Ueberwachungssignal angeschlossen
#define uebsig2_OUT 13  // hier wird das linke Ueberwachungssignal angeschlossen
#define UT1_Status_OUT 14  // hier wird das rechte UT-Kontrolllicht angeschlossen
#define UT2_Status_OUT 15  // hier wird das linke UT-Kontrolllicht angeschlossen

#define Schranke_1_PIN 0                // Erste Schranke
#define Schranke_2_PIN 1                // Zweite Schranke
#define Schranke_3_PIN 2
#define Schranke_4_PIN 3
#define Schranke_5_PIN 4
#define Schranke_6_PIN 5
#define Schranke_7_PIN 6
#define Schranke_8_PIN 7

#define speaker_PIN 9                // Anschluss fuer den Lautsprecher

#define SD_ChipSelectPin 10          // Chip-Select fuer die SD-Karte


// Eingangspins
#define ET_PIN A0                    // Einschalttaster
#define UT1_PIN A3                   // Unwirksamkeitstaster rechts
#define UT2_PIN A4                   // Unwirksamkeitstaster links
#define AT_PIN A1                    // Ausschalttaster
#define RS_PIN A2                    // Rangierschalter
#define Strom1_PIN 5                 // Stromsensor rechts
#define Strom2_PIN 6                 // Stromsensor links
#define LS_PIN A5                    // Freimeldung des Bahnübergangs

// Eingänge definieren
Button ET = Button(ET_PIN, LOW);            // Einschalttaster
Button UT1 = Button(UT1_PIN, LOW);          // Unwirksamkeitstaster rechts
Button UT2 = Button(UT2_PIN, LOW);          // Unwirksamkeitstaster links
Button AT = Button(AT_PIN, LOW);            // Ausschalttaster
Button RS = Button(RS_PIN, LOW);            // Rangierschalter
Button Strom1 = Button(Strom1_PIN, LOW);    // Stromsensor rechts
Button Strom2 = Button(Strom2_PIN, LOW);    //Stromsensor links
Button LS = Button(LS_PIN, HIGH);           // Freimeldung des Bahnübergangs

// Variablen fuer das Blinklicht
int brightnessBlink1 = 0;      // Helligkeit fuer das erste Blinklicht
int brightnessBlink2 = 0;      // Helligkeit fuer das zweite Blinklicht
int Index1 = 0;
int Index2 = 0;
int periode = 1000;             // Dauer einer Blinkperiode
boolean blinkLichtAn = false;  // Blinklicht beim Start ausgeschaltet
#define AN 4095                // PWM auf maximal
#define AUS 4096               // PWM aus

//const PROGMEM byte sinus_tabelle[] = {
//    0,   0,   0,   0,   0,   0,   0,   0,
//    0,   1,   1,   1,   1,   1,   1,   1,
//    1,   1,   1,   1,   1,   2,   2,   2,
//    2,   2,   2,   2,   2,   3,   3,   3,
//    3,   3,   3,   4,   4,   4,   4,   5,
//    5,   5,   5,   6,   6,   6,   6,   7,
//    7,   8,   8,   8,   9,   9,  10,  10,
//   11,  11,  12,  12,  13,  13,  14,  15,
//   15,  16,  17,  18,  18,  19,  20,  21,
//   22,  23,  24,  25,  26,  28,  29,  30,
//   32,  33,  35,  36,  38,  40,  41,  43,
//   45,  47,  49,  52,  54,  56,  59,  62,
//   64,  67,  70,  73,  77,  80,  84,  88,
//   91,  96, 100, 104, 109, 114, 119, 124,
//  130, 136, 142, 148, 155, 161, 169, 176,
//  184, 192, 201, 210, 219, 229, 239, 250,
//  250, 239, 229, 219, 210, 201, 192, 184,
//  176, 169, 161, 155, 148, 142, 136, 130,
//  124, 119, 114, 109, 104, 100,  96,  91,
//   88,  84,  80,  77,  73,  70,  67,  64,
//   62,  59,  56,  54,  52,  49,  47,  45,
//   43,  41,  40,  38,  36,  35,  33,  32,
//   30,  29,  28,  26,  25,  24,  23,  22,
//   21,  20,  19,  18,  18,  17,  16,  15,
//   15,  14,  13,  13,  12,  12,  11,  11,
//   10,  10,   9,   9,   8,   8,   8,   7,
//    7,   6,   6,   6,   6,   5,   5,   5,
//    5,   4,   4,   4,   4,   3,   3,   3,
//    3,   3,   3,   2,   2,   2,   2,   2,
//    2,   2,   2,   1,   1,   1,   1,   1,
//    1,   1,   1,   1,   1,   1,   1,   0,
//    0,   0,   0,   0,   0,   0,   0,   0
//};

const PROGMEM byte sinus_tabelle[] = {
    0,  0,  0,  0,  1,  1,  1,  2,  2,  3,  4,  5,  5,  6,  7,  9,
   10, 11, 12, 14, 15, 16, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35,
   37, 39, 42, 44, 46, 49, 51, 54, 56, 59, 62, 64, 67, 70, 73, 76,
   78, 81, 84, 87, 90, 93, 96, 99,102,105,108,111,115,118,121,124,
  127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,
  176,178,181,184,187,190,192,195,198,200,203,205,208,210,212,215,
  217,219,221,223,225,227,229,231,233,234,236,238,239,240,242,243,
  244,245,247,248,249,249,250,251,252,252,253,253,253,254,254,254,
  254,254,254,254,253,253,253,252,252,251,250,249,249,248,247,245,
  244,243,242,240,239,238,236,234,233,231,229,227,225,223,221,219,
  217,215,212,210,208,205,203,200,198,195,192,190,187,184,181,178,
  176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,
  127,124,121,118,115,111,108,105,102, 99, 96, 93, 90, 87, 84, 81,
   78, 76, 73, 70, 67, 64, 62, 59, 56, 54, 51, 49, 46, 44, 42, 39,
   37, 35, 33, 31, 29, 27, 25, 23, 21, 20, 18, 16, 15, 14, 12, 11,
   10,  9,  7,  6,  5,  5,  4,  3,  2,  2,  1,  1,  1,  0,  0,  0
};

/* Zeitkonstanten

Hier kann das Zeitverhalten des BUEs angepasst werden.

- GrundstellerPeriod: Zeit, nach der der BUE nach dem Schliessen wieder geoeffnet wird, wenn er nicht befahren wurde.
- UTtimeoutPeriod:    Zeit, nachder der Stromsensor wieder aktiviert wird, wenn er mit UT deaktiviert wurde.
- FreiPeriod:         Zeit, die hoechstens zwischen zwei Ausloesungen der Lichtschranke auf dem BUE liegen darf. Nach deren Ablauf wird der BUE frei gemeldet.
- RuhezeitPeriod:     Zeit, waehrend der der Stromsensor nach dem Oeffnen deaktiviert ist. So soll verhindert werden, dass der abfahrende Zug den BUE wieder schliesst.

*/

long GrundstellerPeriod = 300000;             // Zeit bis zur Grundstellung des Bahnübergangs (öffnen)
long UTtimeoutPeriod = 120000;               // Zeit bis zum automatischen Rücknehmen der Unwirksamkeitstaste
long FreiPeriod = 4000;                    // Zeit, innerhalb der die LS erneut ausgeloest werden muss. Sonst wird BUE frei gemeldet
long RuhezeitPeriod = 120000;                // Zeit, innerhalb derer der BUE nach Oeffnen nicht auf den Stromsensor reagiert


unsigned long currentMillis = 0;      // Zaehlerstand beim letzten Blinken
unsigned long previousUT1timeoutMillis = 0;  // speichert die Zeit, zu der der UT1 (rechts) Timeout zuletzt ausgeloest wurde
unsigned long previousUT2timeoutMillis = 0;
unsigned long previousGrundstellerMillis = 0; 
unsigned long previousFreiMillis = 0;
unsigned long previousRuhezeitMillis = 0;

// Variablen für den Zustandsautomaten
int state = 1;                        // aktueller Zustand
int next_state = 1;                   // nächster Zustand

// Tonausgabe
TMRpcm tmrpcm;                       // Objekt fuer Tonerzeugung

// Definieren des PWM-Treibers
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
//#define SERVOMIN  350 // this is the 'minimum' pulse length count (out of 4096)
//#define SERVOMAX  560 // this is the 'maximum' pulse length count (out of 4096)

boolean schrankenHoch = true;

class Schranke
{
   int Pin;             // Pin, an den die Schranke angeschlossen ist
//MR   int PosUnten;        // Position der Stellung "Unten"
//MR   int PosOben;         // Position der Stellung "Oben"
   int Position;   // Aktuelle Position der Schranke
   int Aenderung;   // Änderung der Position je Druchgang
//MR   int updateInterval;   // wie schnell wird die Schranke bewegt
   unsigned long previousSchrankeMillis = 0;  // letzte Aktualisierung der Position
  SchrankenDaten& daten;

public:
  // MR
  Schranke(int port, SchrankenDaten& daten)
    : daten(daten)
  // /MR
  {
    Pin = port;
//MR    PosUnten = 350;
//MR    PosOben = 560;
//MR    Position = PosUnten;  // geht so nicht mehr, da EEPROM erst nach Konstruktor gelesen wird, daher in init verschoben
//MR    updateInterval = 20; 
    Aenderung = 3;
  }

  // MR
  void init() 
  {
    Position = daten.PosUnten;
  }
  // /MR

  void gotoUnten()
  {
    pwm.setPWM(Pin, 0, /*MR PosUnten*/daten.PosUnten);
    DEBUG_PRINTLN(/*MR PosUnten*/daten.PosUnten);
  }

  void setUnten(int increment)
  { 
    if ((/*MR PosUnten*/daten.PosUnten + increment <= 4095) && (/*MR PosUnten*/daten.PosUnten + increment >= 0) && (/*MR PosUnten*/daten.PosUnten + increment <= /*MR PosOben*/daten.PosOben)) {
      /*MR PosUnten*/daten.PosUnten += increment;
    }
  }

  void gotoOben()
  {
    pwm.setPWM(Pin, 0, /*MR PosOben*/daten.PosOben);
    DEBUG_PRINTLN(/*MR PosOben*/daten.PosOben);
  }
  
  void setOben(int increment)
  {
    if ((/*MR PosOben*/daten.PosOben + increment <= 4095) && (/*MR PosOben*/daten.PosOben + increment >= 0) && (/*MR PosOben*/daten.PosOben + increment >= /*MR PosUnten*/daten.PosUnten)) {
      /*MR PosOben*/daten.PosOben += increment;
    }
  }

  void Sweep()
  {
    if ((millis() - previousSchrankeMillis) > /*MR updateInterval*/daten.updateInterval)
    {
      previousSchrankeMillis = millis();
      Position += Aenderung;
      pwm.setPWM(Pin, 0, Position);
      DEBUG_PRINTLN(/*MR updateInterval*/daten.updateInterval);
      if ((Position > /*MR PosOben*/daten.PosOben) || (Position < /*MR PosUnten*/daten.PosUnten))  // Ende der Bewegung erreicht
      {
        // Richtung umkehren
        Aenderung = -Aenderung;
      }
    }
  }

  void setInterval(int increment)
  {
    if (/*MR updateInterval*/daten.updateInterval + increment >= 1) {
      /*MR updateInterval*/daten.updateInterval += increment;
    }
  }
  
  void bewegen()
  {
    if ((millis() - previousSchrankeMillis) > /*MR updateInterval*/daten.updateInterval) {
      previousSchrankeMillis = millis();

      if (schrankenHoch == true) {
        Aenderung = abs(Aenderung);
        if (Position < /*MR PosOben*/daten.PosOben) {
          // weiter hoch fahren
          Position += Aenderung;
          pwm.setPWM(Pin, 0, Position);
          DEBUG_PRINTLN("hoch");
        }  
      }
      else {
        Aenderung = abs(Aenderung);
        if (Position > /*MR PosUnten*/daten.PosUnten) {
          // weiter runter fahren
          Position -= Aenderung;
          pwm.setPWM(Pin, 0, Position);
          DEBUG_PRINTLN("runter");  
        }
      }
    }
  }

  
};


// MR
Schranke schranke_1(Schranke_1_PIN, eepromData.payload.schranke[0]);
Schranke schranke_2(Schranke_2_PIN, eepromData.payload.schranke[1]);
Schranke schranke_3(Schranke_3_PIN, eepromData.payload.schranke[2]);
Schranke schranke_4(Schranke_4_PIN, eepromData.payload.schranke[3]);
Schranke schranke_5(Schranke_5_PIN, eepromData.payload.schranke[4]);
Schranke schranke_6(Schranke_6_PIN, eepromData.payload.schranke[5]);
Schranke schranke_7(Schranke_7_PIN, eepromData.payload.schranke[6]);
Schranke schranke_8(Schranke_8_PIN, eepromData.payload.schranke[7]);
// /MR

int servoSetupState = 1;         // Startzustand für die Einstellung der Servowerte

boolean servoSetup = false;       // TRUE, wenn die Servopositionen eingestellt werden sollen


// Funktion zum Blinken
void blinkLicht() {
  
  if (blinkLichtAn == true) {
    // Es soll geblinkt werden
    //brightnessBlink1 = 2048+2047*cos(2*PI/periode*currentMillis);
    //brightnessBlink2 = 2048+2047*sin(2*PI/periode*(currentMillis));
    Index1 = ((currentMillis * 256 / periode) % 255);
    brightnessBlink1 = 16 * pgm_read_byte(&sinus_tabelle[Index1]);
  
    Index2 = (((currentMillis * 256 / periode) + 127) % 255);
    brightnessBlink2 = 16 * pgm_read_byte(&sinus_tabelle[Index2]);
    
    pwm.setPWM(blink1_OUT, 0, brightnessBlink1);
    pwm.setPWM(blink2_OUT, 0, brightnessBlink2);
    
    pwm.setPWM(uebsig1_OUT, 0, brightnessBlink1);
    pwm.setPWM(uebsig2_OUT, 0, brightnessBlink1);
  }
  else {
    // Blinklichter ausschalten
    pwm.setPWM(blink1_OUT, 0, AUS);
    pwm.setPWM(blink2_OUT, 0, AUS);
    
    pwm.setPWM(uebsig1_OUT, 0, AUS);
    pwm.setPWM(uebsig2_OUT, 0, AUS);
  }
}



void enableUT1timeout() {
 previousUT1timeoutMillis = currentMillis; 
}

boolean checkUT1timeout() {
  // Timer fuer UT1-Timeout laeuft
  if (currentMillis - previousUT1timeoutMillis >= UTtimeoutPeriod) {
    return true; // UT timeout wird ausgeloest
  }
  else {
   return false; // kein UT timeout 
  }
}

void enableUT2timeout() {
 previousUT2timeoutMillis = currentMillis; 
}

boolean checkUT2timeout() {
  // Timer fuer UT2-Timeout laeuft
  if (currentMillis - previousUT2timeoutMillis >= UTtimeoutPeriod) {
    return true; // UT timeout wird ausgeloest
  }
  else {
   return false; // kein UT timeout 
  }
}

void enableGrundsteller() {
  previousGrundstellerMillis = currentMillis;
}

boolean checkGrundsteller() {
 if (currentMillis - previousGrundstellerMillis >= GrundstellerPeriod) {
   return true; // Grundsteller Timeout ausgeloest
 }
 else {
   return false; // kein Grundsteller Timeout
 }
}

void enableRuhezeit() {
  previousRuhezeitMillis = currentMillis;
}

boolean checkRuhezeit() {
  if (currentMillis - previousRuhezeitMillis >= RuhezeitPeriod) {
    return true; // Ruhezeit Timeout ausgeloest
  }
  else {
    return false; // kein Ruhezeit Timeout
  }
}

void enableFrei() {
 previousFreiMillis = currentMillis; 
}

boolean checkFrei() {
  if (LS.onPress()) {
   previousFreiMillis = currentMillis;
   return false;
  }
 else if (LS.isPressed()) {
   previousFreiMillis = currentMillis;
   return false;
  }   
 else if (currentMillis - previousFreiMillis >= FreiPeriod) {
  return true;
 }
 else {
   return false;
 }
}

// MR

void StoreToEEPROM() {
  eepromData.crc = CRC32::calculate(&eepromData.payload, sizeof(eepromData.payload));
  EEPROM.put(0, eepromData);
}

// /MR

void setup() {
  // put your setup code here, to run once:
  
  pinMode(ET_PIN, INPUT_PULLUP);
  pinMode(UT1_PIN, INPUT_PULLUP);
  pinMode(UT2_PIN, INPUT_PULLUP);
  pinMode(AT_PIN, INPUT_PULLUP);
  pinMode(RS_PIN, INPUT_PULLUP);
  pinMode(Strom1_PIN, INPUT_PULLUP);
  pinMode(Strom2_PIN, INPUT_PULLUP);
  pinMode(LS_PIN, INPUT_PULLUP);

  tmrpcm.speakerPin = speaker_PIN;            // Audioausgang definieren
  
  if (!SD.begin(SD_ChipSelectPin)) {  // Nachsehen, ob die SD-Karte da ist und initialisiert werden kann:
    return;                           // nichts mehr tun
    }
  tmrpcm.setVolume(1);              // Lautstaerke setzen

  #ifdef DEBUG
    Serial.begin(9600);
    DEBUG_PRINTLN("BÜ-Steuerung");
  #endif

  pwm.begin();
  pwm.setPWMFreq(60);        // Analoge Servos mit etwa 60Hz betreiben


  if (digitalRead(UT1_PIN)==LOW || digitalRead(UT2_PIN)==LOW) {   // Wird beim Start eine der UT-Tasten gedrückt, so beginnt die Routine für die Einstellung der Servo Positionen
    DEBUG_PRINT("Servo Setup starten");
    servoSetup = true;
    pwm.setPWM(blink4_OUT, 0, 4000);
    }
  else {
    DEBUG_PRINT("Normaler Betrieb");
    pwm.setPWM(blink3_OUT, 0, 1000);
  }

  // MR
  EEPROM.get(0, eepromData);
  uint32_t crc = CRC32::calculate(&eepromData.payload, sizeof(eepromData.payload));
  if (crc != eepromData.crc) {
    // TODO: invalid CRC, reset values to defaults/go to setup mode/do whatever...
    //       For now, initialize with defauls that were previously set in the constructor
    //       Note: initialization could also be done by calling a member of Schranke which initialized the data there
    for (int i = 0; i < 8; ++i) {
      DEBUG_PRINT("CRC falsch!");
      eepromData.payload.schranke[i].PosUnten = 350;
      eepromData.payload.schranke[i].PosOben = 560;
      eepromData.payload.schranke[i].updateInterval = 20;
    }
    // TODO: here we could set servoSetup to true (auto-setup on EEPROM read failure)
  }
  schranke_1.init();
  schranke_2.init();
  schranke_3.init();
  schranke_4.init();
  schranke_5.init();
  schranke_6.init();
  schranke_7.init();
  schranke_8.init();
  // /MR

}


void loop() {
  // put your main code here, to run repeatedly:
  // Buttonwerte abfragen
  ET.listen();
  UT1.listen();
  UT2.listen();
  AT.listen();
  RS.listen();
  Strom1.listen();
  Strom2.listen();
  LS.listen();
    
  // Aktuelle Zeit speichern
  currentMillis = millis();

  if (servoSetup == true) {
    // Einstellung der Servo Positionen

    switch(servoSetupState) {
      case 1:
        DEBUG_PRINT("1unten");
        pwm.setPWM(blink1_OUT, 0, AN); // an
        pwm.setPWM(blink2_OUT, 0, AUS); // aus
        pwm.setPWM(blink3_OUT, 0, AUS); // aus
        pwm.setPWM(blink4_OUT, 0, AUS); // aus
        pwm.setPWM(uebsig1_OUT, 0, AUS); // aus
        pwm.setPWM(uebsig2_OUT, 0, AUS); // aus
        pwm.setPWM(UT1_Status_OUT, 0, AUS); // aus
        pwm.setPWM(UT2_Status_OUT, 0, AUS); // aus
  
        if (ET.onPress()) {schranke_1.setUnten(+5);};
        if (AT.onPress()) {schranke_1.setUnten(-5);};
        
        schranke_1.gotoUnten();
  
        if (UT1.onPress()) {
          servoSetupState=2;
          }
        break;
  
      case 2:
        DEBUG_PRINT("1oben");
  
        if (ET.onPress()) {schranke_1.setOben(+5);};
        if (AT.onPress()) {schranke_1.setOben(-5);};
  
        schranke_1.gotoOben();
        
        if (UT1.onPress()) {
          servoSetupState=3;
          }  
        break;
  
      case 3:
        DEBUG_PRINT("1speed");
  
        if (ET.onPress()) {schranke_1.setInterval(+1);};
        if (AT.onPress()) {schranke_1.setInterval(-1);};
  
        schranke_1.Sweep();
        
        if (UT1.onPress()) {
          servoSetupState=4;
          }  
  
        break;
  
        
      case 4:
        DEBUG_PRINT("2unten");
        pwm.setPWM(blink1_OUT, 0, AUS); // aus
        pwm.setPWM(blink2_OUT, 0, AN); // an
        pwm.setPWM(blink3_OUT, 0, AUS); // aus
        pwm.setPWM(blink4_OUT, 0, AUS); // aus
        pwm.setPWM(uebsig1_OUT, 0, AUS); // aus
        pwm.setPWM(uebsig2_OUT, 0, AUS); // aus
        pwm.setPWM(UT1_Status_OUT, 0, AUS); // aus
        pwm.setPWM(UT2_Status_OUT, 0, AUS); // aus

   
        if (ET.onPress()) {schranke_2.setUnten(+5);};
        if (AT.onPress()) {schranke_2.setUnten(-5);};
        
        schranke_2.gotoUnten();
  
        if (UT1.onPress()) {
          servoSetupState=5;
          }
   
        break;
  
      case 5:
        DEBUG_PRINT("2oben");
  
        if (ET.onPress()) {schranke_2.setOben(+5);};
        if (AT.onPress()) {schranke_2.setOben(-5);};
  
        schranke_2.gotoOben();
        
        if (UT1.onPress()) {
          servoSetupState=6;
          }  
  
        break;
  
      case 6:
        DEBUG_PRINT("2speed");
  
        if (ET.onPress()) {schranke_2.setInterval(+1);};
        if (AT.onPress()) {schranke_2.setInterval(-1);};
  
        schranke_2.Sweep();
        
        if (UT1.onPress()) {
          servoSetupState=7;
          }  
        break;

      case 7:
        DEBUG_PRINTLN("End Setup");
        // MR
        StoreToEEPROM();
        // /MR
        // Zurück zum normalen Betrieb
        servoSetup = false;
        servoSetupState = 1;
        state = 1;
        pwm.setPWM(blink1_OUT, 0, AUS); // aus
        pwm.setPWM(blink2_OUT, 0, AUS); // aus
        pwm.setPWM(blink3_OUT, 0, AUS); // aus
        pwm.setPWM(blink4_OUT, 0, AUS); // aus
        pwm.setPWM(uebsig1_OUT, 0, AUS); // aus
        pwm.setPWM(uebsig2_OUT, 0, AUS); // aus
        pwm.setPWM(UT1_Status_OUT, 0, AUS); // aus
        pwm.setPWM(UT2_Status_OUT, 0, AUS); // aus   
    }  

  }
  else {
    // Normaler Betrieb
 
   switch (state) {
    case 1:
      // Zustand: Ruhezustand
      
      // Zustandsausgabe
      //bin_out(state);
      
      if (RS.isPressed()) {
        next_state = 3;
        blinkLichtAn = true;
      }
      else if (ET.onPress() || Strom1.onPress() || Strom1.isPressed() || Strom2.onPress() || Strom2.isPressed()) {
        next_state = 4;
        enableGrundsteller();
        blinkLichtAn = true;
      }
       else if (UT1.onPress()) {
        next_state = 2;
        enableUT1timeout(); // UT Timeout aktivieren
      }
      else {
       next_state = 1; 
      }
      break;
    case 2:
      // Zustand: Stromsensor aus
      // Zustandsausgabe
      //bin_out(state);      
      
     if (RS.isPressed()) {
        next_state = 3;
        blinkLichtAn = true;
        pwm.setPWM(UT1_Status_OUT, 0, AUS);
        pwm.setPWM(UT2_Status_OUT, 0, AUS);
      }
      else if (ET.onPress()) {
        next_state = 4;
        enableGrundsteller();
        blinkLichtAn = true;
        pwm.setPWM(UT1_Status_OUT, 0, AUS);
        pwm.setPWM(UT2_Status_OUT, 0, AUS);
      }
      else if (checkUT1timeout()==true) {
        next_state = 1;
        pwm.setPWM(UT1_Status_OUT, 0, AUS);
        pwm.setPWM(UT2_Status_OUT, 0, AUS);
      }
      else {
        next_state = 2;
        pwm.setPWM(UT1_Status_OUT, 0, AN);
        pwm.setPWM(UT2_Status_OUT, 0, AN);
      }
      
      break;
    case 3:
      // Zustand: BÜ schließen rangieren
      // Zustandsausgabe
      //bin_out(state);
      
      next_state = 5;
      
      tmrpcm.loop(1);
      tmrpcm.play("ding.wav");
      
      schrankenHoch = false;
      
      break;
    case 4:
      // Zustand: BÜ schließen normal
      // Zustandsausgabe
      //bin_out(state);
      
      next_state = 6;
      
      tmrpcm.loop(1);
      tmrpcm.play("ding.wav");
      
      schrankenHoch = false;
      
      break;
    case 5:
      // Zustand: BÜ geschlossen rangieren
      // Zustandsausgabe
      //bin_out(state);
      
      if (RS.isPressed()) {
        next_state = 5;
      }
      else {
       next_state = 10; 
      }
      
      break;
    case 6:
      // Zustand: BÜ geschlossen normal
      // Zustandsausgabe
      //bin_out(state);
      
      if (RS.isPressed()) {
        next_state = 5;
      }
      else if (LS.onPress()) {
       next_state = 7; 
       enableFrei();
      }
      else if (AT.onPress() || checkGrundsteller() == true) {
        next_state = 8;
      }
      else {
        next_state = 6;
      }
      break;
    case 7:
      // Zustand: BÜ geschlossen, befahren
      // Zustandsausgabe
      //bin_out(state);
    
      if (RS.isPressed()) {
        next_state = 5;
      }
      else if (checkFrei()==true) {
       next_state = 8; 
      }
      else {
        next_state = 7;
      }
      break;
    case 8:
      // Zustand: BÜ öffnen
      // Zustandsausgabe
      //bin_out(state);
      
      next_state = 9;
      blinkLichtAn = false;
      
      schrankenHoch = true;
      
      tmrpcm.stopPlayback();
      
      enableRuhezeit();  // Starte Timer fuer Ruhezeit
      
      break;
    case 9:
      // Zustand: Ruhezeit
      // Zustandsausgabe
      //bin_out(state);
      
      if (RS.isPressed()) {
        next_state = 3;
        blinkLichtAn = true;
      }
      else if (ET.onPress()) {
        next_state = 4;
        enableGrundsteller();
        blinkLichtAn = true;
      }
      else if (checkRuhezeit()==true) {
        next_state = 1;
      }
      else if (UT1.onPress()) {
        next_state = 2;
        enableUT1timeout(); // UT1 Timeout aktivieren
      }
      else {
        next_state = 9;
      }
        
      break;
    case 10:
      // Zustand: BUE oeffnen, rangieren
      // Zustandausgabe
      //bin_out(state);
      
      next_state = 1;
      blinkLichtAn = false;
      
      schrankenHoch = true;
      
      tmrpcm.stopPlayback();
      
      break;
  }
  // Zustandswechsel durchfuehren
  state = next_state;
 
  blinkLicht();
  schranke_1.bewegen();
  schranke_2.bewegen();
  
  }

}
