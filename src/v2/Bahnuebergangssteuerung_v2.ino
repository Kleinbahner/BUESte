/* Bahnuebergangsteuerung v2.1.4 (C) Jens Becker
 *  
 *  Unterstützt nur die Hardware Version 03/21 der Bahnübergangssteuerung BÜSte
 *  
 *  Treiber fuer den PWM-Chip von Adafruit https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/archive/master.zip
 * 
 *  Debouncing der Tasten mit der Bounce2 Bibliothek https://github.com/thomasfredericks/Bounce2
 *
 *  Audiowiedergabe über DFPlayer mini MP3-Modul mit Bibliothek PowerBroker2 https://github.com/PowerBroker2/DFPlayerMini_Fast
 * 
 *  DFlayerMini_Fast library requires FireTimer lib https://github.com/PowerBroker2/FireTimer
 *
 *  storing EEPROM values requires CRC32 library  *https://downloads.arduino.cc/libraries/github.com/bakercp/CRC32-2.0.0.zip // Disabled DKap
*/
//##########################################################################
//#
//#   Version History:
//#
//#-------------------------------------------------------------------------
//#
//#  Version: 2.1.4 date: 2022/02/27 author: JBec
//#
//# Added:
//#   - Changed Deboucing-Lib to Bounce2 
//#     https://github.com/thomasfredericks/Bounce2
//#   - Removed forced Servo setup if EEPROM data was wrong
//#   - LED test at startup
//#
//#-------------------------------------------------------------------------
//#
//#  Version: 2.1.3 date: 2021/10/16 author: DKap
//#
//# Added:
//#   - DFFast driver, for other models DFmini is required
//#     --> #define DFFast
//#
//#-------------------------------------------------------------------------
//#
//#  Version: 2.1.2 date: 2021/04/20 author: DKap
//#
//# Added:
//#   - delay Schranke implemended
//#
//#-------------------------------------------------------------------------
//#
//#  Version: 2.1.2 date: 2021/04/19 author: DKap
//#
//# Added:
//#   - UT2 also detected on servo setup
//#   - Servo setup also scalable up to 4 servos with indication of which 
//#     setting on the other 4 LED
//#
//#-------------------------------------------------------------------------
//#
//#  Version: 2.1.2 date: 2021/04/18 author: DKap
//#
//# Added:
//#   - Servo variables scalable by changing NUM_OF_SCHRANKEN, no code
//#     change necessary
//#
//#-------------------------------------------------------------------------
//#
//#  Version: 2.1.2 date: 2021/04/15 author: DKap
//#
//# Added:
//#   - removed CRC from EEPROM storing
//#
//#-------------------------------------------------------------------------


const int VERSION_MAJOR = 2;
const int VERSION_MINOR = 1;
const int VERSION_PATCH = 4;

//==========================================================================
//
//    I N C L U D E S
//
//==========================================================================

#include <EEPROM.h>

#include <avr/pgmspace.h> // Wird zum Ablegen von Werten im FLASH benoetigt

// PWM-Driver for PCA9685
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Debouncing of inputs
#include <Bounce2.h>

// CRC32 1.1.0 (Christopher Baker)
#include <CRC32.h>

// Uncomment if sound is not working
//#define DFFast

// DFPlayer mini fast
#ifdef DFFast
  #include <DFPlayerMini_Fast.h>
#else
  #include <DFRobotDFPlayerMini.h>
#endif
#include <SoftwareSerial.h>


//==========================================================================
//
//    D E F I N I T I O N S
//
//==========================================================================

// enable Debug-Output on serial interface
#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINT(x)     Serial.print (x)
 #define DEBUG_PRINTDEC(x)     Serial.print (x, DEC)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTLN(x) 
#endif

#define SW_VERSION 1  // For EEPROM data, data will be set to default value is not the same

// Ausgangspins
#define blink1_OUT 15     // hier wird das erste Blinklicht angeschlossen
#define blink2_OUT 14     // hier wird das zweite Blinklicht angeschlossen
#define blink3_OUT 13     // hier wird das dritte Blinklicht angeschlossen
#define blink4_OUT 12     // hier wird das vierte Blinklicht angeschlossen
#define uebsig1_OUT 11    // hier wird das rechte Ueberwachungssignal angeschlossen
#define uebsig2_OUT 10    // hier wird das linke Ueberwachungssignal angeschlossen
#define UT1_Status_OUT 9  // hier wird das rechte UT-Kontrolllicht angeschlossen
#define UT2_Status_OUT 8  // hier wird das linke UT-Kontrolllicht angeschlossen

#define NUM_OF_SCHRANKEN 4

//Verbindungspins zum DFPlayer mini
#define DFPlayer_RX A3
#define DFPlayer_TX A2

// Eingangspins
#define ET_PIN 5                    // Einschalttaster
#define UT1_PIN 8                   // Unwirksamkeitstaster rechts
#define UT2_PIN 9                   // Unwirksamkeitstaster links
#define AT_PIN 6                    // Ausschalttaster
#define RS_PIN 7                    // Rangierschalter
#define Strom1_PIN 12               // Stromsensor rechts
#define Strom2_PIN 11               // Stromsensor links
#define LS_PIN 10                   // Freimeldung des Bahnübergangs (Lichtschranke)

#define volume_PIN A6                // Anschluss fuer den Poti zur Lautstärkeregelung

// Define Button objects
Bounce2::Button ET = Bounce2::Button();            // Einschalttaster
Bounce2::Button UT1 = Bounce2::Button();          // Unwirksamkeitstaster rechts
Bounce2::Button UT2 = Bounce2::Button();          // Unwirksamkeitstaster links
Bounce2::Button AT = Bounce2::Button();            // Ausschalttaster
Bounce2::Button RS = Bounce2::Button();            // Rangierschalter
Bounce2::Button Strom1 = Bounce2::Button();    // Stromsensor rechts
Bounce2::Button Strom2 = Bounce2::Button();    //Stromsensor links
Bounce2::Button LS = Bounce2::Button();           // Freimeldung des Bahnübergangs


// Variablen fuer das Blinklicht
int brightnessBlink1 = 0;      // Helligkeit fuer das erste Blinklicht
int brightnessBlink2 = 0;      // Helligkeit fuer das zweite Blinklicht
int Index1 = 0;
int Index2 = 0;
const int periode = 1000;      // Dauer einer Blinkperiode
boolean blinkLichtAn = false;  // Blinklicht beim Start ausgeschaltet
#define AN 4095                // PWM auf maximal
#define AUS 4096               // PWM aus

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
- ServoDownIncrement  Zeit, die ein Servo später startet als der erste Vorgang um verzögerte Schranken zu erzeugen

*/

const long GrundstellerPeriod = 300000;             // Zeit bis zur Grundstellung des Bahnübergangs (öffnen)
const long UTtimeoutPeriod = 120000;               // Zeit bis zum automatischen Rücknehmen der Unwirksamkeitstaste
const long FreiPeriod = 4000;                    // Zeit, innerhalb der die LS erneut ausgeloest werden muss. Sonst wird BUE frei gemeldet
const long RuhezeitPeriod = 120000;                // Zeit, innerhalb derer der BUE nach Oeffnen nicht auf den Stromsensor reagiert
const long ServoDownIncrement = 250;            // time of an increment of delay for Schranken to go down later than the start time

unsigned long currentMillis = 0;      // Zaehlerstand beim letzten Blinken
unsigned long previousUT1timeoutMillis = 0;  // speichert die Zeit, zu der der UT1 (rechts) Timeout zuletzt ausgeloest wurde
unsigned long previousUT2timeoutMillis = 0;
unsigned long previousGrundstellerMillis = 0; 
unsigned long previousFreiMillis = 0;
unsigned long previousRuhezeitMillis = 0;

boolean SchrankenSetDelay = false;
boolean demoTheDelay = false;

boolean isUT1TimeoutOn = false;
boolean isUT2TimeoutOn = false;

// Variablen für den Zustandsautomaten
uint8_t state = 1;                        // aktueller Zustand
uint8_t next_state = 1;                   // nächster Zustand
uint8_t setup_servo = 0;                  //servo that is being setup

// Lautstärke
int sensorValue = 0;  //value read from volume_PIN from 0 to 1023
uint8_t volumeValue = 0;  //value for volume from 0 to 30

//Ansteuerung DFPlayer mini
SoftwareSerial mySoftwareSerial(DFPlayer_RX, DFPlayer_TX); // RX, TX

#ifdef DFFast
  DFPlayerMini_Fast myDFPlayer;
#else
  DFRobotDFPlayerMini myDFPlayer;
#endif

// Definieren des PWM-Treibers
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

typedef struct  {
   int      PosUnten;        // Position der Stellung "Unten"
   int      PosOben;         // Position der Stellung "Oben"  
   int      updateInterval;   // wie schnell wird die Schranke bewegt
   uint8_t  downdelay;  //delayed start of servo down in 1 sec increments  
} SchrankenDaten;

typedef struct {
  SchrankenDaten schranke[NUM_OF_SCHRANKEN];  // in dieser Konfiguration maximal acht Schranken
} EEPROMPayload ;

typedef struct {
  uint8_t swversion;
  EEPROMPayload payload;
  uint32_t crc;
}  EEPROMData ;

EEPROMData eepromData;

boolean schrankenHoch = true;

typedef struct 
{
   uint8_t Pin;             // Pin, an den die Schranke angeschlossen ist
   int Position;            // Aktuelle Position der Schranke
   int Aenderung;           // Änderung der Position je Druchgang
   unsigned long previousSchrankeMillis = 0;  // letzte Aktualisierung der Position
  //SchrankenDaten& daten;
} Schranke;

Schranke Schrankenset[NUM_OF_SCHRANKEN];

void schrankeInit(uint8_t port) 
{
  Schrankenset[port].Pin = port;
  Schrankenset[port].Aenderung = 3;
  Schrankenset[port].Position = eepromData.payload.schranke[port].PosUnten;
}

void setEEPROMDefaults() {
  for (uint8_t i = 0; i < NUM_OF_SCHRANKEN; ++i) {
      eepromData.payload.schranke[i].PosUnten = 350;
      eepromData.payload.schranke[i].PosOben = 560;
      eepromData.payload.schranke[i].updateInterval = 15;
      eepromData.payload.schranke[i].downdelay = 0; 
    }
    eepromData.swversion = SW_VERSION;
}

void showEEPROMValues(uint8_t pin)
  {
  DEBUG_PRINT("SW-Version: ");
  DEBUG_PRINTLN(eepromData.swversion);
  DEBUG_PRINT("Pin       : ");
  DEBUG_PRINTLN(pin);
  DEBUG_PRINT("Up        : ");
  DEBUG_PRINTLN(eepromData.payload.schranke[pin].PosUnten);
  DEBUG_PRINT("Down      : ");
  DEBUG_PRINTLN(eepromData.payload.schranke[pin].PosOben);
  DEBUG_PRINT("Speed     : ");
  DEBUG_PRINTLN(eepromData.payload.schranke[pin].updateInterval);
  DEBUG_PRINT("Downdelay : ");
  DEBUG_PRINTLN(eepromData.payload.schranke[pin].downdelay);

}

void gotoUnten(uint8_t pin)
{
  pwm.setPWM(Schrankenset[pin].Pin, 0, eepromData.payload.schranke[pin].PosUnten);
  //DEBUG_PRINTLN(eepromData.payload.schranke[pin].PosUnten);
}

void setUnten(uint8_t pin, int increment)
{ 
  if ((eepromData.payload.schranke[pin].PosUnten + increment <= 4095) && (eepromData.payload.schranke[pin].PosUnten + increment >= 0) && (eepromData.payload.schranke[pin].PosUnten + increment <= eepromData.payload.schranke[pin].PosOben)) {
    eepromData.payload.schranke[pin].PosUnten += increment;
  }
}

void gotoOben(uint8_t pin)
{
  pwm.setPWM(Schrankenset[pin].Pin, 0, eepromData.payload.schranke[pin].PosOben);
  //DEBUG_PRINTLN(eepromData.payload.schranke[pin].PosOben);
}

void setOben(uint8_t pin, int increment)
{
  if ((eepromData.payload.schranke[pin].PosOben + increment <= 4095) && (eepromData.payload.schranke[pin].PosOben + increment >= 0) && (eepromData.payload.schranke[pin].PosOben + increment >= eepromData.payload.schranke[pin].PosUnten)) {
    eepromData.payload.schranke[pin].PosOben += increment;
  }
}


void Sweep(uint8_t pin)
{
  if ((millis() - Schrankenset[pin].previousSchrankeMillis) > eepromData.payload.schranke[pin].updateInterval)
  {
    Schrankenset[pin].previousSchrankeMillis = millis();
    Schrankenset[pin].Position += Schrankenset[pin].Aenderung;
    pwm.setPWM(Schrankenset[pin].Pin, 0, Schrankenset[pin].Position);
    if ((Schrankenset[pin].Position > eepromData.payload.schranke[pin].PosOben) || (Schrankenset[pin].Position < eepromData.payload.schranke[pin].PosUnten))  // Ende der Bewegung erreicht
    {
      // Richtung umkehren
      Schrankenset[pin].Aenderung = -Schrankenset[pin].Aenderung;
    }
  }
}

void demoDelay(uint8_t pin)
{
  if (demoTheDelay == true) {//show the delay when it was changed
    pwm.setPWM(UT2_Status_OUT, 0, AN); //show the timed delay
    pwm.setPWM(UT2_Status_OUT, 0, AUS);
    delay(eepromData.payload.schranke[pin].downdelay * ServoDownIncrement);
    pwm.setPWM(UT2_Status_OUT, 0, AN);
    demoTheDelay = false;
  }
}
void setInterval(uint8_t pin, int increment)
{
  if (eepromData.payload.schranke[pin].updateInterval + increment >= 1) {
    eepromData.payload.schranke[pin].updateInterval += increment;
  }
}

void setdowndelay(uint8_t pin, int increment)
{
  if (eepromData.payload.schranke[pin].downdelay + increment >= 1) {
    eepromData.payload.schranke[pin].downdelay += increment;
  }
}

void bewegen(uint8_t pin)
{
   if (millis()  > (eepromData.payload.schranke[pin].updateInterval + Schrankenset[pin].previousSchrankeMillis)) { //is it time to update servo?
 
    Schrankenset[pin].previousSchrankeMillis = millis();

    if (schrankenHoch == true) {
      Schrankenset[pin].Aenderung = abs(Schrankenset[pin].Aenderung);
      if (Schrankenset[pin].Position < eepromData.payload.schranke[pin].PosOben) {
        // weiter hoch fahren
        Schrankenset[pin].Position += Schrankenset[pin].Aenderung;
        pwm.setPWM(Schrankenset[pin].Pin, 0, Schrankenset[pin].Position);
      }  
    }
    else {
      Schrankenset[pin].Aenderung = abs(Schrankenset[pin].Aenderung);
      if (Schrankenset[pin].Position > eepromData.payload.schranke[pin].PosUnten) {
        // weiter runter fahren
        Schrankenset[pin].Position -= Schrankenset[pin].Aenderung;
        pwm.setPWM(Schrankenset[pin].Pin, 0, Schrankenset[pin].Position);
      }
    }
  } 
}


void schrankenbewegen() {
  if (SchrankenSetDelay == true) {
      for (uint8_t i = 0; i < NUM_OF_SCHRANKEN; ++i){
        Schrankenset[i].previousSchrankeMillis = (millis() + (eepromData.payload.schranke[i].downdelay * ServoDownIncrement));
      }
      SchrankenSetDelay = false;
    }
  for (uint8_t i = 0; i < NUM_OF_SCHRANKEN; ++i){
    bewegen(i);
   }
}

uint8_t servoSetupState = 1;         // Startzustand für die Einstellung der Servowerte

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
    pwm.setPWM(blink3_OUT, 0, brightnessBlink1);
    pwm.setPWM(blink4_OUT, 0, brightnessBlink2);
    
    pwm.setPWM(uebsig1_OUT, 0, brightnessBlink1);
    pwm.setPWM(uebsig2_OUT, 0, brightnessBlink2);
  }
  else {
    // Blinklichter ausschalten
    pwm.setPWM(blink1_OUT, 0, AUS);
    pwm.setPWM(blink2_OUT, 0, AUS);
    pwm.setPWM(blink3_OUT, 0, AUS);
    pwm.setPWM(blink4_OUT, 0, AUS);
    
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

void setUTTimeout(uint8_t UTno) { 
    switch(UTno) {
      case 1 :
        enableUT1timeout(); // UT 1 Timeout aktivieren
        pwm.setPWM(UT1_Status_OUT, 0, AN);
        isUT1TimeoutOn = true;
        //DEBUG_PRINTDEC(UTno);
        break;
      case 2 :
        enableUT2timeout(); // UT 2 Timeout aktivieren
        pwm.setPWM(UT2_Status_OUT, 0, AN);
        isUT2TimeoutOn = true;
        //DEBUG_PRINTDEC(UTno);
      break;
    }
}

void clearUTTimeout(uint8_t UTno) { 
   switch(UTno) {
      case 1 :
        pwm.setPWM(UT1_Status_OUT, 0, AUS);
        isUT1TimeoutOn = false;
        //DEBUG_PRINTDEC(UTno);
      break;
      case 2 :
        pwm.setPWM(UT2_Status_OUT, 0, AUS);
        isUT2TimeoutOn = false;
        //DEBUG_PRINTDEC(UTno);
      break;
    }
    //DEBUG_PRINTLN("");
}

void checkUT() {//check if timeouts are firing and act
  if ((checkUT1timeout()==true) & (isUT1TimeoutOn == true)) {
        clearUTTimeout(1);
  }
  if ((checkUT2timeout()==true) & (isUT2TimeoutOn == true)) {
        clearUTTimeout(2);
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
  if (LS.pressed()) {
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

void StoreToEEPROM() {
  eepromData.swversion = SW_VERSION;
  eepromData.crc = CRC32::calculate(&eepromData.payload, sizeof(eepromData.payload));
  EEPROM.put(0, eepromData);
}


//**********************************************************************
//  setup
//
void setup() {
  // put your setup code here, to run once:

  // BUTTON SETUP
  ET.attach(ET_PIN, INPUT_PULLUP);            // Einschalttaster
  ET.setPressedState(LOW);
  UT1.attach(UT1_PIN, INPUT_PULLUP);          // Unwirksamkeitstaster rechts
  UT1.setPressedState(LOW);
  UT2.attach(UT2_PIN, INPUT_PULLUP);           // Unwirksamkeitstaster links
  UT2.setPressedState(LOW);
  AT.attach(AT_PIN, INPUT_PULLUP);             // Ausschalttaster
  AT.setPressedState(LOW);
  RS.attach(RS_PIN, INPUT_PULLUP);             // Rangierschalter
  RS.setPressedState(LOW);
  Strom1.attach(Strom1_PIN, INPUT_PULLUP);     // Stromsensor rechts
  Strom1.setPressedState(LOW);
  Strom2.attach(Strom2_PIN, INPUT_PULLUP);     //Stromsensor links
  Strom2.setPressedState(LOW);

// Only one of the following two options should be active
// Lichtschranke HIGH active (e.g. IS 471)
  LS.attach(LS_PIN, INPUT);            // Lichtschranke HIGH active
  LS.setPressedState(HIGH);            // Lichtschranke HIGH active
// Lichtschranke LOW active
//  LS.attach(LS_PIN, INPUT_PULLUP);     // Lichtschranke LOW active
//  LS.setPressedState(LOW);             // Lichtschranke LOW active


  pinMode(volume_PIN, INPUT);

  #ifdef DEBUG
    Serial.begin(115200);
    DEBUG_PRINT("BÜ v");
    DEBUG_PRINT(VERSION_MAJOR);
    DEBUG_PRINT(".");
    DEBUG_PRINT(VERSION_MINOR);
    DEBUG_PRINT(".");
    DEBUG_PRINT(VERSION_PATCH);
    DEBUG_PRINTLN(" (C) JBec, DKap");
  #endif

// Verbindung zum DFPlayer mini
  mySoftwareSerial.begin(9600);
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with DFPlayer
   while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
 
  //read in volume poti and set DFPlayer volume
  sensorValue = analogRead(volume_PIN);
  volumeValue = map(sensorValue, 0, 1023, 0, 30); // Map analog input to volume of DFPlayer

  myDFPlayer.volume(volumeValue);  //Set volume value. From 0 to 30
  
  // Start PWM for outputs
  pwm.begin();
  pwm.setPWMFreq(60);

  // Test lights
  pwm.setPWM(blink1_OUT, 0, AN); 
  pwm.setPWM(blink2_OUT, 0, AN);
  pwm.setPWM(blink3_OUT, 0, AN);
  pwm.setPWM(blink4_OUT, 0, AN);
  pwm.setPWM(uebsig1_OUT, 0, AN);
  pwm.setPWM(uebsig2_OUT, 0, AN);
  pwm.setPWM(UT1_Status_OUT, 0, AN);
  pwm.setPWM(UT2_Status_OUT, 0, AN);
  delay(1000);
  pwm.setPWM(blink1_OUT, 0, AUS); 
  pwm.setPWM(blink2_OUT, 0, AUS);
  pwm.setPWM(blink3_OUT, 0, AUS);
  pwm.setPWM(blink4_OUT, 0, AUS);
  pwm.setPWM(uebsig1_OUT, 0, AUS);
  pwm.setPWM(uebsig2_OUT, 0, AUS);
  pwm.setPWM(UT1_Status_OUT, 0, AUS);
  pwm.setPWM(UT2_Status_OUT, 0, AUS);

  if (digitalRead(UT1_PIN)==LOW || digitalRead(UT2_PIN)==LOW) {   // Wird beim Start eine der UT-Tasten gedrückt, so beginnt die Routine für die Einstellung der Servo Positionen
    servoSetup = true;
    setup_servo = 99; 
    //set LEDS to setup indication 
    }
  else {
    DEBUG_PRINTLN("Normal Operation");
  }

  EEPROM.get(0, eepromData);
  uint32_t crc = CRC32::calculate(&eepromData.payload, sizeof(eepromData.payload));
  if (eepromData.swversion != SW_VERSION) { //reset if version update of EEPROM data 
    DEBUG_PRINTLN("Reset EEPROM data");
    
    // invalid CRC, reset values to defaults/go to setup mode/do whatever...
    setEEPROMDefaults();
    //servoSetup = true; 
    //setup_servo = 99; 
  } 

 for (uint8_t i = 0; i < NUM_OF_SCHRANKEN; ++i) {
  schrankeInit(i);
  showEEPROMValues(i);
 }

  myDFPlayer.stop();
}

void loop() {
  // put your main code here, to run repeatedly:
  // Buttonwerte abfragen

  ET.update();
  UT1.update();
  UT2.update();
  AT.update();
  RS.update();
  Strom1.update();
  Strom2.update();
  LS.update();
  
  //read in volume poti and set DFPlayer volume
  sensorValue = analogRead(volume_PIN);
  if (!(volumeValue == map(sensorValue, 0, 1023, 0, 30))) { //only on change
    volumeValue = map(sensorValue, 0, 1023, 0, 30);
    //DEBUG_PRINT("Snd: volume: ");
    //DEBUG_PRINTLN(volumeValue);
    myDFPlayer.volume(volumeValue);  //Set volume value. From 0 to 30
  }
  
  // Aktuelle Zeit speichern
  currentMillis = millis();

  if (servoSetup == true) {
    // Einstellung der Servo Positionen
    //DEBUG_PRINTLN(servoSetupState);
    //DEBUG_PRINTLN(setup_servo);
   switch(setup_servo) {
    case 99: {
      DEBUG_PRINTLN("Servo setup");
      //DEBUG_PRINTLN("Servo 0:"); 
      DEBUG_PRINTLN("up");
      servoSetupState = 1;
      setup_servo = 0;
      pwm.setPWM(blink1_OUT, 0, AN); // an
      pwm.setPWM(blink2_OUT, 0, AUS); // aus
      pwm.setPWM(blink3_OUT, 0, AUS); // aus
      pwm.setPWM(blink4_OUT, 0, AUS); // aus
      pwm.setPWM(uebsig1_OUT, 0, AN); // an Indicate "down" setup
      pwm.setPWM(uebsig2_OUT, 0, AUS); // aus
      pwm.setPWM(UT1_Status_OUT, 0, AUS); // aus
      pwm.setPWM(UT2_Status_OUT, 0, AUS); // aus 
      }
    case 0 ... NUM_OF_SCHRANKEN:
    //case 1:
    //case 2:
    //case 3:
      {
      //DEBUG_PRINT("Servo ");
      //DEBUG_PRINTLN(setup_servo);
      switch(servoSetupState) {
        case 1:
          //DEBUG_PRINTLN("down");
          
          if (ET.pressed()) {DEBUG_PRINTLN("+5");setUnten(setup_servo,+5);};
          if (AT.pressed()) {DEBUG_PRINTLN("-5");setUnten(setup_servo,-5);};
          
          gotoUnten(setup_servo);
    
          if (UT1.pressed() || UT2.pressed() ) {
            DEBUG_PRINTLN("up");
            servoSetupState=2;
            pwm.setPWM(uebsig1_OUT, 0, AUS); // aus
            pwm.setPWM(uebsig2_OUT, 0, AN); // indicate "up" setup
            }
          break;
    
        case 2:
          
          if (ET.pressed()) {DEBUG_PRINTLN("+5");setOben(setup_servo,+5);};
          if (AT.pressed()) {DEBUG_PRINTLN("-5");setOben(setup_servo,-5);};
    
          gotoOben(setup_servo);
          
          if (UT1.pressed() || UT2.pressed() ) {
            DEBUG_PRINTLN("speed");
            servoSetupState=3;
            pwm.setPWM(uebsig2_OUT, 0, AUS); // aus
            pwm.setPWM(UT1_Status_OUT, 0, AN); // indicate speed setup
            }  
          break;
    
        case 3:
          
          if (ET.pressed()) {DEBUG_PRINTLN("+1");setInterval(setup_servo,+1);};
          if (AT.pressed()) {DEBUG_PRINTLN("-1");setInterval(setup_servo,-1);};
    
          Sweep(setup_servo);
          
          if (UT1.pressed() || UT2.pressed() ) {
            DEBUG_PRINTLN("delay");
            servoSetupState=4;
            pwm.setPWM(UT1_Status_OUT, 0, AUS); // aus
            pwm.setPWM(UT2_Status_OUT, 0, AN);  // delay setup step
            }  
          break;
          
        case 4:
          
          if (ET.released()) {DEBUG_PRINTLN("+1");demoTheDelay = true;setdowndelay(setup_servo,+1);};
          if (AT.released()) {DEBUG_PRINTLN("-1");demoTheDelay = true;setdowndelay(setup_servo,-1);};
          
          demoDelay(setup_servo);
          
          if (UT1.pressed() || UT2.pressed() ) { //after step 4, go to the next servo or to the end of the setup
            servoSetupState=1;
            setup_servo++;
            if (setup_servo >= NUM_OF_SCHRANKEN) {setup_servo = 98;} //go to save the setup
            pwm.setPWM(uebsig1_OUT, 0, AN); // down stage setup
            pwm.setPWM(UT2_Status_OUT, 0, AUS); // aus
            switch (setup_servo) {
              //case 0:
              //  pwm.setPWM(blink1_OUT, 0, AN); // aus
              //break;
              case 1:
                pwm.setPWM(blink1_OUT, 0, AUS); 
                pwm.setPWM(blink2_OUT, 0, AN); // aus
              break;
              case 2:
                pwm.setPWM(blink2_OUT, 0, AUS); 
                pwm.setPWM(blink3_OUT, 0, AN); // aus
              break;
              case 3:  
                pwm.setPWM(blink3_OUT, 0, AUS); 
                pwm.setPWM(blink4_OUT, 0, AN); // aus
              break;
              }  
            }
            
          break;
      }
      break;
      }
    case 98: {
      //save the setup for all servos
    
      // clean up setup
        DEBUG_PRINTLN("End Setup");
        StoreToEEPROM();
        servoSetup = false;
        servoSetupState = 1;
        setup_servo = 0;
        state = 1;
        pwm.setPWM(blink1_OUT, 0, AUS); // aus
        pwm.setPWM(blink2_OUT, 0, AUS); // aus
        pwm.setPWM(blink3_OUT, 0, AUS); // aus
        pwm.setPWM(blink4_OUT, 0, AUS); // aus
        pwm.setPWM(uebsig1_OUT, 0, AUS); // aus
        pwm.setPWM(uebsig2_OUT, 0, AUS); // aus
        pwm.setPWM(UT1_Status_OUT, 0, AUS); // aus
        pwm.setPWM(UT2_Status_OUT, 0, AUS); // aus
      break;
    }
   }
  }
  else {
    // Normaler Betrieb
 
   switch (state) {
    case 1:
      // Zustand: Ruhezustand
      
      // Zustandsausgabe
      //DEBUG_PRINTLN("Ruhezustand");
      
      if (RS.isPressed()) {
      //if (RS.pressed()) {
        next_state = 2;
        DEBUG_PRINTLN("1 Ruhezustand --> 2 BÜ schließen rangieren");
        blinkLichtAn = true;
      }
      else if (ET.pressed()) {
        next_state = 3;
        DEBUG_PRINTLN("1 Ruhezustand --> 3 BÜ schließen normal, ET");
        enableGrundsteller();
        blinkLichtAn = true;
      }
      else if ((isUT1TimeoutOn==false)&&(Strom1.pressed() || Strom1.isPressed() )) {
        next_state = 3;
        DEBUG_PRINTLN("1 Ruhezustand --> 3 BÜ schließen normal, Strom1");
        enableGrundsteller();
        blinkLichtAn = true;
      }
      else if ((isUT2TimeoutOn==false)&&(Strom2.pressed() || Strom2.isPressed() )) {
        next_state = 3;
        DEBUG_PRINTLN("1 Ruhezustand --> 3 BÜ schließen normal, Strom2");
        enableGrundsteller();
        blinkLichtAn = true;
      }
      else if (UT1.pressed()) {
        setUTTimeout(1);
        next_state = 1;
        DEBUG_PRINTLN("1 Ruhezustand --> 1 Ruhezustand, UT1");
      }
      else if (UT2.pressed()) {
        setUTTimeout(2);
        next_state = 1;
        DEBUG_PRINTLN("1 Ruhezustand --> 1 Ruhezustand, UT2");
      }
      
      else {
       next_state = 1; 
      }
      break;
    case 2:
      // Zustand: BÜ schließen rangieren
      // Zustandsausgabe
      //bin_out(state);
      //DEBUG_PRINTLN("BÜ schließen rangieren");
      
      next_state = 4;
      DEBUG_PRINTLN("2 BÜ schließen rangieren --> 4 BÜ geschlossen rangieren");
      
      //start sound
      myDFPlayer.loop(1);
      #ifdef DFFast
        myDFPlayer.startRepeatPlay();
      #endif
      schrankenHoch = false;
      SchrankenSetDelay = true;
      
      break;
    case 3:
      // Zustand: BÜ schließen normal
      // Zustandsausgabe
      //DEBUG_PRINTLN("BÜ schließen normal");
      
      next_state = 5;
      DEBUG_PRINTLN("3 BÜ schließen normal -->5 BÜ geschlossen normal");
      
      //start sound
      //DEBUG_PRINTLN("Snd: loop");
      myDFPlayer.loop(1);
      #ifdef DFFast
        myDFPlayer.startRepeatPlay();
      #endif
      
      schrankenHoch = false;
      SchrankenSetDelay = true;
      
      break;
    case 4:
      // Zustand: BÜ geschlossen rangieren
      // Zustandsausgabe
      //bin_out(state);
      //DEBUG_PRINTLN("BÜ geschlossen rangieren");
      
      if (RS.isPressed()) {
        next_state = 4;
        //DEBUG_PRINTLN("4-->4");
      }
      else {
       next_state = 9; 
       DEBUG_PRINTLN("4 BÜ geschlossen rangieren --> 9 BÜ öffnen rangieren");
      }
      
      break;
    case 5:
      // Zustand: BÜ geschlossen normal
      // Zustandsausgabe
      //DEBUG_PRINTLN("BÜ geschlossen normal");
      
      if (RS.isPressed()) {
        next_state = 4;
        DEBUG_PRINTLN("5 BÜ geschlossen normal --> 4 BÜ geschlossen rangieren");
      }
      else if (LS.pressed()) {
       next_state = 6; 
       DEBUG_PRINTLN("5 BÜ geschlossen normal --> 6 BÜ geschlossen befahren");
       enableFrei();
      }
      else if (AT.pressed() || checkGrundsteller() == true) {
        next_state = 7;
        DEBUG_PRINTLN("5 BÜ geschlossen normal --> 7 BÜ öffnen, AT");
      }
      else {
        next_state = 5;
      }
      break;
    case 6:
      // Zustand: BÜ geschlossen, befahren
      // Zustandsausgabe
      //DEBUG_PRINTLN("BÜ geschlossen, befahren");
    
      if (RS.isPressed()) {
        next_state = 4;
        DEBUG_PRINTLN("6 BÜ geschlossen befahren --> 4 BÜ geschlossen rangieren, RS");
      }
      else if (checkFrei()==true) {
       next_state = 7; 
       DEBUG_PRINTLN("6 BÜ geschlossen befahren --> 7 BÜ öffnen");
      }
      else {
        next_state = 6;
      }
      break;
    case 7:
      // Zustand: BÜ öffnen
      // Zustandsausgabe
      //DEBUG_PRINTLN("BÜ öffnen");
      
      next_state = 8;
      DEBUG_PRINTLN("7 BÜ öffnen --> 8 Ruhezeit");
      blinkLichtAn = false;
      
      schrankenHoch = true;
      
      // stop sound
      //myDFPlayer.play(1);
      myDFPlayer.stop();
      
      enableRuhezeit();  // Starte Timer fuer Ruhezeit
      
      break;
    case 8:
      // Zustand: Ruhezeit
      // Zustandsausgabe
      //DEBUG_PRINTLN("Ruhezeit");
      
      if (RS.isPressed()) {
        next_state = 2;
        DEBUG_PRINTLN("8 Ruhezeit -->2 BÜ schließen rangieren, RS");
        blinkLichtAn = true;
      }
      else if (ET.pressed()) {
        next_state = 3;
        DEBUG_PRINTLN("8 Ruhezeit -->3 BÜ schließen normal, ET");
        enableGrundsteller();
        blinkLichtAn = true;
      }
      else if (checkRuhezeit()==true) {
        next_state = 1;
        DEBUG_PRINTLN("8 Ruhezeit --> 1 Ruhezustand");
      }
      else if (UT1.pressed()) {
        setUTTimeout(1);
      }
      else if (UT2.pressed()) {
        setUTTimeout(2);
      }
      else {
        next_state = 8;
      }
        
      break;
    case 9:
      // Zustand: BUE oeffnen, rangieren
      // Zustandausgabe
      //DEBUG_PRINTLN("BÜ öffnen, rangieren");
      
      next_state = 1;
      DEBUG_PRINTLN("9 BÜ öffnen rangieren --> 1 Ruhezustand");
      blinkLichtAn = false;
      
      schrankenHoch = true;
      
      // stop sound
      myDFPlayer.stop();
      
      break;
  }
  
  // Zustandswechsel durchfuehren
  state = next_state;
 
  blinkLicht();
  schrankenbewegen();
  checkUT();
  }

}
