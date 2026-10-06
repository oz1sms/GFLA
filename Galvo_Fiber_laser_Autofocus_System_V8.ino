/*
* Galvo Fiber Laser Autofocus Control System Ver. 8 (Open-Source Edition)
* Hardware ESP32-WROOM-DA, OLED SSD1309 display i2c (128x64), EC11 Encoder, Endstop
* Hurtig-knapper: +10 Button, -10 Button, +1 Button, -1 Button
* IDE: Arduino IDE
*
*  
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <DIYables_OLED_SSD1309.h>
#include <AccelStepper.h>
#include <EEPROM.h>


// Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define I2C_SDA       21
#define I2C_SCL       22

DIYables_OLED_SSD1309 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// EEPROM Size
#define EEPROM_SIZE 256

// Pin definitioner
#define ENCODER_CLK   32
#define ENCODER_DT    14
#define ENCODER_SW    13

#define BTN_PLUS_10   16
#define BTN_MINUS_10  17
#define BTN_PLUS_1    18
#define BTN_MINUS_1   19

#define ENDSTOP_TOP  33

#define STEP_PIN     25
#define DIR_PIN      26
#define EN_PIN       27

// --- ESP32 Fodpedal og Relæ opsætning ---
const int buttonPin = 4;  
const int relayPin = 23;  
// (Vi har ikke længere brug for lastDebounceTime og debounceDelay variablerne)

// --- 3D Auto Engraving Variabler ---
int engraveAutoMinutes = 0;
int engraveAutoSeconds = 0;
float engraveAutoDepth = 0.00;
int engraveAutoSetupStep = 0; // 0=Minutter, 1=Sekunder, 2=Dybde, 3=Vent på pedal, 4=Kører, 5=Færdig
unsigned long engraveStartTime = 0;
long engraveStartPos = 0;


// Stepper opsætning (4mm pr omgang @ 16 microsteps = 3200 steps)
const float STEPS_PER_MM = 800.0;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Antal opsætninger - Udvidet til 6 linser
const int TOTAL_LENSES = 3; // kan max indeholde 6 linser

// Navne på de 6 standardlinser
const char* lensNames[TOTAL_LENSES] = { "70x70", "110x110", "150x150" };
//const char* lensNames[TOTAL_LENSES] = { "70x70", "110x110", "150x150", "175x175", "200x200", "300x300" };

const int TOTAL_COLORS = 5;
// Navne på de 5 farver (Inkluderer nu en Off)
const char* colorNames[TOTAL_COLORS] = { "Off", "1", "2", "3", "4" };

// Menustrukturer 
enum State { 
  STATUS_SCREEN, MAIN_MENU, MAT_HEIGHT_MENU, LENS_MENU, ENGRAVE_3D_SELECT, ENGRAVE_3D_MENU, ENGRAVE_3D_AUTO, COLOR_MENU, 
  SETUP_MENU, CALIBRATE_HEIGHT_MENU, CALIBRATE_LENS_MENU, COLOR_OFFSET_MENU, 
  WIZARD_WELCOME, WIZARD_HOMING, WIZARD_HEIGHT, WIZARD_LENSES 
};

State currentState = STATUS_SCREEN;
int engraveMenuIndex = 0; // 0 = Manual, 1 = Auto

// Globale variabler (Gemmes i EEPROM)
struct Settings {
  bool isCalibrated;                // True hvis førstegangsopsætning er gennemført
  int selectedLens;                 // 0 til 5
  float materialHeight;             // i mm
  float colorOffsets[TOTAL_COLORS]; // 4 forskellige farve offsets i mm
  int selectedColorIndex;           // Aktiv farveprofil (0-3)
  long currentPositionSteps;        // Gemmer den fysiske Z-position i steps
  float maxPhysicalHeight;          // Den kalibrerede max højde fra top-endestop til bund
  float lensFocusDistances[TOTAL_LENSES]; // Fokusværdier for de 6 linser
} settings;

const int EEPROM_ADDR = 0;

// Encoder variabler
int lastClkVal;
unsigned long lastButtonPress = 0;
int menuIndex = 0;

const int MENU_ITEMS_COUNT = 6;
const char* menuItems[MENU_ITEMS_COUNT] = {
  "1. Material Height",
  "2. 3D Engraving",
  "3. Steel Colors",
  "4. Select Lens",
  "5. Home z-axis",
  "6. Setup / Calibrate"
};


int setupMenuIndex = 0;
const int SETUP_MENU_COUNT = 5; 
const char* setupMenuItems[SETUP_MENU_COUNT] = {
  "1. Calibrate Max H",
  "2. Edit Lens Focus",
  "3. Edit Color Offsets",
  "4. Run Setup Wizard",
  "5. Back / Exit"
};
// Variabler til den nye farvemenu (Disse manglede før)
int calibrateLensSelectIndex = 0; 
int calibrateColorSelectIndex = 1; // Tilføjet
bool editingLensValue = false;
bool editingColorValue = false;    // Tilføjet
int wizardLensIndex = 0; 


// Prototype funktioner
void updateDisplay();
void saveSettings();
void moveToCalculatedFocus();
void executeHome(bool silent = false);
void handleEncoder();
void handleButtons();
void handleQuickButtons();
void moveWithWaitMessage();
void hardware_stepper_run_block();

void IRAM_ATTR handleFootPedal() {
  // Vent 10.000 mikrosekunder (10 ms) på at kontakten falder helt til ro.
  // Dette løser "hænge-fast" problemet uden at misse den sidste tilstand.
  delayMicroseconds(10000); 
  
  // Aflæs den stabile tilstand
  bool isPressed = (digitalRead(buttonPin) == LOW);
  
  if (isPressed) {
    digitalWrite(relayPin, LOW);  // TÆNDER relæet (Active-Low)
  } else {
    digitalWrite(relayPin, HIGH); // SLUKKER relæet (Active-Low)
  }
}



void setup() {
  Serial.begin(115200);
  Serial.println("System starter...");
  
  // Pins konfiguration
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(ENDSTOP_TOP, INPUT_PULLUP);
  pinMode(EN_PIN, OUTPUT);
  
  pinMode(BTN_PLUS_10, INPUT_PULLUP);
  pinMode(BTN_MINUS_10, INPUT_PULLUP);
  pinMode(BTN_PLUS_1, INPUT_PULLUP);
  pinMode(BTN_MINUS_1, INPUT_PULLUP);

  digitalWrite(EN_PIN, LOW); // Aktiver TMC2160
  lastClkVal = digitalRead(ENCODER_CLK);

  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1500);
// 1. Sæt relæet op, og SLUK det med det samme (HIGH = slukket på dit relæ)
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); 

  // 2. Sæt fodpedalen op
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleFootPedal, CHANGE);
  
  // Initialiser OLED
  if(!display.begin(SSD1309_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1309 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextColor(SSD1309_WHITE);

  // Initialiser EEPROM
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("EEPROM fejl!");
  } else {
    EEPROM.get(EEPROM_ADDR, settings);

  
    // Tjek om EEPROM er tom, korrupt, eller om maskinen aldrig er blevet kalibreret
    if (settings.isCalibrated != true || isnan(settings.materialHeight) || settings.selectedLens < 0 || settings.selectedLens >= TOTAL_LENSES) {
      Serial.println("Ingen gyldige data fundet. Starter First-Time Setup Wizard...");
      
      settings.isCalibrated = false;
      settings.selectedLens = 0;
      settings.materialHeight = 0.0;
      settings.selectedColorIndex = 0;
      settings.currentPositionSteps = 0;
      settings.maxPhysicalHeight = 130.0; // Standard startværdi for test
      
      // Standard start-fokusværdier for de 6 linser
      settings.lensFocusDistances[0] = 100.0;  // 70x70
      settings.lensFocusDistances[1] = 160.0;  // 110x110
      settings.lensFocusDistances[2] = 210.0;  // 150x150
      settings.lensFocusDistances[3] = 254.0;  // 175x175
      settings.lensFocusDistances[4] = 290.0;  // 200x200
      settings.lensFocusDistances[5] = 420.0;  // 300x300

      for(int i = 0; i < TOTAL_COLORS; i++) {
        settings.colorOffsets[i] = 0.0;
      }
      
      currentState = WIZARD_WELCOME; // Tving brugeren ind i guiden
    } else {
      stepper.setCurrentPosition(settings.currentPositionSteps);
      currentState = STATUS_SCREEN;
    }
  }
  
  updateDisplay();
  Serial.println("System klar!");
}

void loop() {
  // --- 3D Auto Engraving Logik ---
  if (currentState == ENGRAVE_3D_AUTO) {
    if (engraveAutoSetupStep == 3) {
      if (digitalRead(relayPin) == LOW) {
        engraveStartPos = stepper.currentPosition();
        engraveStartTime = millis();
        
        float totalSteps = engraveAutoDepth * STEPS_PER_MM;
        float totalSeconds = (engraveAutoMinutes * 60.0) + engraveAutoSeconds;
        
        if (totalSeconds > 0 && totalSteps > 0) {
          float engraveSpeed = totalSteps / totalSeconds;
          stepper.setSpeed(engraveSpeed); 
          engraveAutoSetupStep = 4; // Skift til "Kører"
          updateDisplay(); 
        } else {
          engraveAutoSetupStep = 6; // Ugyldig tid/dybde - hop til færdig
          updateDisplay();
        }
      }
    } 
    else if (engraveAutoSetupStep == 4) {
      // Sikkerhed hvis pedalen slippes før tid
      /* 
      if (digitalRead(relayPin) == HIGH) {
         stepper.setMaxSpeed(3000); 
         stepper.moveTo(engraveStartPos); // Sæt mål til startposition
         engraveAutoSetupStep = 5; // Skift til "Kører tilbage"
         updateDisplay();
         return; 
      }
      */

      unsigned long totalTimeMs = (engraveAutoMinutes * 60UL + engraveAutoSeconds) * 1000UL;
      unsigned long elapsed = millis() - engraveStartTime;
      
      if (elapsed <= totalTimeMs) {
        stepper.runSpeed(); // Kører langsomt ned
      } else {
        // Tiden er gået, start returnering!
        stepper.setMaxSpeed(3000); 
        stepper.moveTo(engraveStartPos); // Bed motoren køre tilbage til start
        engraveAutoSetupStep = 5; // Ny tilstand: Kører tilbage
        updateDisplay();
      }
    }
    else if (engraveAutoSetupStep == 5) {
      // Kører hurtigt tilbage til startpositionen
      if (stepper.distanceToGo() != 0) {
        stepper.run(); // Normal kørsel med acceleration op ad igen
      } else {
        engraveAutoSetupStep = 6; // Helt færdig status
        updateDisplay();
      }
    } 
    else {
      // Normal motorstyring i indtastning eller når den er 'Færdig'
      stepper.run();
    }
  } else {
    stepper.run(); 
  }

  handleEncoder();
  handleButtons();
  handleQuickButtons();
}

void saveSettings() {
  settings.currentPositionSteps = stepper.currentPosition();
  EEPROM.put(EEPROM_ADDR, settings);
  EEPROM.commit();
  Serial.println("Indstillinger autosaved til EEPROM.");
}

/*------------------------------------------------------------------------------------------------------------------------------*/
void handleEncoder() {  
  int clkVal = digitalRead(ENCODER_CLK);  
  if (clkVal != lastClkVal && clkVal == LOW) {  
    int direction = (digitalRead(ENCODER_DT) != clkVal) ? 1 : -1;  
    
    switch(currentState) {  
      case STATUS_SCREEN:
        // EC11 gør ingenting på hovedskærmen (STATUS_SCREEN)
        break;

      case MAIN_MENU:  
        menuIndex = constrain(menuIndex + direction, 0, MENU_ITEMS_COUNT - 1);  
        break;  
        
      case MAT_HEIGHT_MENU:  
        settings.materialHeight = constrain(settings.materialHeight + (direction * 0.05), 0.0, settings.maxPhysicalHeight);  
        moveToCalculatedFocus();  
        break;  
       
      case WIZARD_HEIGHT:  
        settings.maxPhysicalHeight = constrain(settings.maxPhysicalHeight + (direction * 0.1), 50.0, 600.0);  
        break;  
       
      case WIZARD_LENSES:  
        settings.lensFocusDistances[wizardLensIndex] = constrain(settings.lensFocusDistances[wizardLensIndex] + (direction * 0.5), 0.0, 530.0);  
        break;  
       
      case LENS_MENU:  
        settings.selectedLens = constrain(settings.selectedLens + direction, 0, TOTAL_LENSES - 1);  
        break;  
        
      case ENGRAVE_3D_MENU: {  
        long steps = direction * (0.05 * STEPS_PER_MM);  
        stepper.move(steps);  
        break;  
      }  
      
      case ENGRAVE_3D_SELECT:
        engraveMenuIndex = constrain(engraveMenuIndex + direction, 0, 1);
        break;
        
      case ENGRAVE_3D_AUTO:
        if (engraveAutoSetupStep == 0) {
          engraveAutoMinutes = constrain(engraveAutoMinutes + direction, 0, 59);
          } else if (engraveAutoSetupStep == 1) {
            engraveAutoSeconds = constrain(engraveAutoSeconds + direction, 0, 59);
          } else if (engraveAutoSetupStep == 2) {
          // Justerer med 0.05 mm pr. klik. Begrænset til max 5mm.
          engraveAutoDepth = constrain(engraveAutoDepth + (direction * 0.05), 0.0, 5.0);
        }
      break;

      case COLOR_MENU:  
        settings.selectedColorIndex = constrain(settings.selectedColorIndex + direction, 0, TOTAL_COLORS - 1);  
        moveToCalculatedFocus();
        break;  
       
      case SETUP_MENU:  
        setupMenuIndex = constrain(setupMenuIndex + direction, 0, SETUP_MENU_COUNT - 1);  
        break;  
       
      case CALIBRATE_HEIGHT_MENU:  
        settings.maxPhysicalHeight = constrain(settings.maxPhysicalHeight + (direction * 0.1), 50.0, 600.0);  
        break;  
       
      case CALIBRATE_LENS_MENU:  
        if (!editingLensValue) {  
          calibrateLensSelectIndex = constrain(calibrateLensSelectIndex + direction, 0, TOTAL_LENSES);  
        } else {  
          settings.lensFocusDistances[calibrateLensSelectIndex] = constrain(settings.lensFocusDistances[calibrateLensSelectIndex] + (direction * 0.1), 0.0, 530.0);  
        }  
        break;  
       
      case COLOR_OFFSET_MENU:  
        if (!editingColorValue) {  
          calibrateColorSelectIndex = constrain(calibrateColorSelectIndex + direction, 1, TOTAL_COLORS);  
        } else {  
          settings.colorOffsets[calibrateColorSelectIndex] = constrain(settings.colorOffsets[calibrateColorSelectIndex] + (direction * 0.05), -3.0, 3.0);  
        }  
        break;  
    }  
    updateDisplay();  
  }  
  lastClkVal = clkVal;  
}
/*-----------------------------------------------------------------------------------------------------------------------------*/
void handleQuickButtons() {
  // Tillad KUN hurtig-knapperne i de 5 specifikke tilstande
  if (currentState != WIZARD_HOMING && 
      currentState != WIZARD_HEIGHT && 
      currentState != WIZARD_LENSES && 
      currentState != MAT_HEIGHT_MENU &&
      currentState != CALIBRATE_HEIGHT_MENU && 
      currentState != CALIBRATE_LENS_MENU) {
    return; // Afbryd straks hvis vi står i en anden menu (f.eks. MAT_HEIGHT, STATUS_SCREEN, etc.)
  }

  float adjustment = 0.0;
  if (digitalRead(BTN_PLUS_10) == LOW)       adjustment = 10.0; 
  else if (digitalRead(BTN_MINUS_10) == LOW) adjustment = -10.0;
  else if (digitalRead(BTN_PLUS_1) == LOW)   adjustment = 1.0;
  else if (digitalRead(BTN_MINUS_1) == LOW)  adjustment = -1.0;

  if (adjustment != 0.0) {
    if (millis() - lastButtonPress > 200) { 
      lastButtonPress = millis();

      switch(currentState) {
        case WIZARD_HOMING:
          // Under Homing ignoreres manuel højdejustering
          break;

        case MAT_HEIGHT_MENU:
          // Juster materialehøjden og flyt steppermotoren fysisk
          settings.materialHeight = constrain(settings.materialHeight + adjustment, 0.0, settings.maxPhysicalHeight);
          moveToCalculatedFocus(); 
        break;

        case WIZARD_HEIGHT:
        case CALIBRATE_HEIGHT_MENU:
          // Juster den maksimale fysiske Z-højde
          settings.maxPhysicalHeight = constrain(settings.maxPhysicalHeight + adjustment, 50.0, 600.0);
          break;

        case WIZARD_LENSES:
          // Juster fokusforholdet for den valgte linse i Wizarden
          settings.lensFocusDistances[wizardLensIndex] = constrain(settings.lensFocusDistances[wizardLensIndex] + adjustment, 0.0, 530.0);
          break;

        case CALIBRATE_LENS_MENU:
          // Juster fokusafstand for den valgte linse i kalibreringsmenuen (når der redigeres)
          if (editingLensValue && calibrateLensSelectIndex < TOTAL_LENSES) {
            settings.lensFocusDistances[calibrateLensSelectIndex] = constrain(settings.lensFocusDistances[calibrateLensSelectIndex] + adjustment, 0.0, 530.0);
          }
          break;
      }

      updateDisplay();
    }
  }
}

/*--------------------------------------------------------------------------------------------------------*/

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1309_WHITE);
  
  // --- BLOKERENDE SPECIELLE SKÆRME FOR WIZARDEN ---
  if (currentState == WIZARD_WELCOME) {
    display.setCursor(15, 5); display.print("GALVO FOCUS OS");
    display.drawFastHLine(0, 15, 128, SSD1309_WHITE);
    display.setCursor(0, 25);  display.print("No calibration found");
    display.setCursor(0, 40);  display.print("Press Encoder to");
    display.setCursor(0, 50);  display.print("start Setup Wizard");
    display.display();
    return;
  }
  if (currentState == WIZARD_HOMING) {
    display.setCursor(20, 20); display.print("WIZARD STEP 1/3");
    display.setCursor(10, 40); display.print("Homing Z-axis...");
    display.display();
    return;
  }
  if (currentState == WIZARD_HEIGHT) {
    display.setCursor(0, 0);   display.print("WIZARD STEP 2/3");
    display.drawFastHLine(0, 10, 128, SSD1309_WHITE);
    display.setCursor(0, 15);  display.print("Measure Top -> Bed:");
    display.setTextSize(2);
    display.setCursor(15, 32); display.print(settings.maxPhysicalHeight, 1); display.print(" mm");
    display.setTextSize(1);
    display.setCursor(0, 55);  display.print("Turn: Adj. | Press: Next");
    display.display();
    return;
  }
  if (currentState == WIZARD_LENSES) {
    display.setCursor(0, 0);   display.print("WIZARD STEP 3/3");
    display.drawFastHLine(0, 10, 128, SSD1309_WHITE);
    display.setCursor(0, 15);  display.print("Enter Focus for Lens");
    display.setCursor(0, 26);  display.print(wizardLensIndex + 1); display.print("/"); display.print(TOTAL_LENSES);
    display.print(" ["); display.print(lensNames[wizardLensIndex]); display.print("]");
    
    display.setTextSize(2);
    display.setCursor(25, 40); display.print(settings.lensFocusDistances[wizardLensIndex], 1); display.print(" mm");
    
    display.setTextSize(1);
    display.display();
    return;
  }

  // --- OPERATIONEL TOP-BAR FOR DE NORMALE MENUER ---
  // (Overskriften udskrives kun hvis vi ELLER IKKE står i SETUP_MENU)
  if (currentState != SETUP_MENU && currentState != ENGRAVE_3D_AUTO)  {
    display.setCursor(0, 0);
    display.print("LENS: ");
    display.print(lensNames[settings.selectedLens]);
    display.print("  COL: "); // Laver et lille mellemrum efter linsen og skriver "COL: "
    display.print(colorNames[settings.selectedColorIndex]); // Udskriver den valgte farve eller "Off"
    display.drawFastHLine(0, 10, 128, SSD1309_WHITE);
  }
  
  switch(currentState) {
    case STATUS_SCREEN: {
      float currentPosFromTopMM = (float)stepper.currentPosition() / STEPS_PER_MM;
      float currentPosMM = settings.maxPhysicalHeight - currentPosFromTopMM;
      
      display.setCursor(0, 15); display.print("Focus Dist:"); 
      display.setCursor(72, 15); display.print(currentPosMM, 2); display.print(" mm");
      
      display.setCursor(0, 28); display.print("Material  :");
      display.setCursor(72, 28); display.print(settings.materialHeight, 2); display.print(" mm");
      
      display.drawFastHLine(0, 42, 128, SSD1309_WHITE);
      // display.setCursor(0, 46); display.print("Turn Adjust Z +/-0.05");
      display.setCursor(0, 56); display.print("Press: Mainmenu");
      break;
    }
    
    case MAIN_MENU:
      for(int i = 0; i < MENU_ITEMS_COUNT; i++) {
        if(i == menuIndex) {
          display.fillRect(0, 12 + (i * 8), 128, 9, SSD1309_WHITE);
          display.setTextColor(SSD1309_BLACK);
        } else {
          display.setTextColor(SSD1309_WHITE);
        }
        display.setCursor(2, 13 + (i * 8));
        display.print(menuItems[i]);
      }
      display.setTextColor(SSD1309_WHITE);
      break;
    
    case MAT_HEIGHT_MENU:
      display.setCursor(10, 20);  display.print("Material Height:");
      display.setTextSize(2);
      display.setCursor(20, 35);  display.print(settings.materialHeight, 2); display.print(" mm");
      break;

    case LENS_MENU:
      display.setCursor(10, 12);  display.print("Select Lens:");
      for(int i = 0; i < TOTAL_LENSES; i++) {
        int yPos = 22 + (i * 7);
        if (yPos > 57) break;
        display.setCursor(5, yPos);
        if(i == settings.selectedLens) display.print("> ");
        else                           display.print("  ");
        display.print(lensNames[i]);
        display.print(" (F"); display.print(settings.lensFocusDistances[i], 1); display.print(")");
      }
      break;

    case COLOR_MENU:
      display.setCursor(1, 15);  display.print("Select Color Profile:");
      display.setTextSize(2);
      display.setCursor(40, 35);
      
      if (settings.selectedColorIndex == 0) {
        display.print("OFF");
      } else {
        display.print("# "); 
        display.print(settings.selectedColorIndex); // Vi har fjernet + 1 her
      }
      break;

case ENGRAVE_3D_SELECT:
      display.setCursor(10, 17); display.print("3D Engraving Mode:");
      display.setCursor(10, 26);
      if (engraveMenuIndex == 0) display.print("> Manual mode");
      else                       display.print("  Manual mode");
      
      display.setCursor(10, 41);
      if (engraveMenuIndex == 1) display.print("> Auto mode");
      else                       display.print("  Auto mode");
      break;

case ENGRAVE_3D_AUTO: {
  display.setCursor(0, 0);
  display.print("3D AUTO ENGRAVING");
  display.drawFastHLine(0, 10, 128, SSD1309_WHITE);
  
  // Viser Minutter
  if (engraveAutoSetupStep == 0) display.setTextColor(SSD1309_BLACK, SSD1309_WHITE);
  else display.setTextColor(SSD1309_WHITE);
  display.setCursor(0, 15);
  display.print("Min: "); display.print(engraveAutoMinutes);
  
  // Viser Sekunder
  if (engraveAutoSetupStep == 1) display.setTextColor(SSD1309_BLACK, SSD1309_WHITE);
  else display.setTextColor(SSD1309_WHITE);
  display.setCursor(64, 15);
  display.print("Sek: "); display.print(engraveAutoSeconds);
  
  // Viser Dybde (mm)
  if (engraveAutoSetupStep == 2) display.setTextColor(SSD1309_BLACK, SSD1309_WHITE);
  else display.setTextColor(SSD1309_WHITE);
  display.setCursor(0, 28);
  display.print("Dybde: "); display.print(engraveAutoDepth, 2); display.print(" mm");

  // Viser bundmenu-hjælper
  display.setTextColor(SSD1309_WHITE);
  display.drawFastHLine(0, 42, 128, SSD1309_WHITE);
  display.setCursor(0, 46);
  
 if (engraveAutoSetupStep < 3) {
    display.print("Drej for at aendre");
    display.setCursor(0, 56); display.print("Tryk Encoder: Naste");
  } else if (engraveAutoSetupStep == 3) {
    display.print("KLAR! Start Laser..");
    display.setCursor(0, 56); display.print("Tryk Fodpedal -> Dyk");
  } else if (engraveAutoSetupStep == 4) {
    display.print("KORER NED...");
  } else if (engraveAutoSetupStep == 5) {
    display.print("KORER TILBAGE..."); // Vises mens Z-aksen kører op igen
  } else if (engraveAutoSetupStep == 6) {
    display.print("FAERDIG!");
    display.setCursor(0, 56); display.print("Tryk for menu");
  }
  break;
}

case ENGRAVE_3D_MENU: {
      // Beregn den nuværende fokusafstand
      float currentPosFromTopMM = (float)stepper.currentPosition() / STEPS_PER_MM;
      float currentPosMM = settings.maxPhysicalHeight - currentPosFromTopMM;

      // Udskriv overskrift (rykket lidt op for at gøre plads)
      display.setCursor(87, 0);   display.print("3D");
      
      // Udskriv fokusafstanden
      display.setCursor(10, 18);  display.print("Focus: ");
      display.print(currentPosMM, 2); 
      display.print(" mm");
      
      // Udskriv instruktioner
      display.setCursor(10, 36);  display.print("Turn: +/- 0.05mm");
      display.setCursor(10, 52);  display.print("Press SW to exit");
      break;
    }
    case SETUP_MENU: {
      display.setCursor(10, 0); 
      display.print("--- SETUP MENU ---");
      display.drawFastHLine(0, 10, 128, SSD1309_WHITE);

      // Udregn hvilke 4 linjer der skal vises (scroll-mekanisme)
      int startIdx = 0;
      if (setupMenuIndex >= 4) {
        startIdx = setupMenuIndex - 3;
      }

      for(int i = 0; i < 4; i++) {
        int itemIdx = startIdx + i;
        if (itemIdx >= SETUP_MENU_COUNT) break;

        int yPos = 14 + (i * 12);

        if(itemIdx == setupMenuIndex) {
          display.fillRect(0, yPos, 128, 11, SSD1309_WHITE);
          display.setTextColor(SSD1309_BLACK);
        } else {
          display.setTextColor(SSD1309_WHITE);
        }

        display.setCursor(2, yPos + 2);
        display.print(setupMenuItems[itemIdx]);
      }
      display.setTextColor(SSD1309_WHITE);
      break;
    }

    case CALIBRATE_HEIGHT_MENU:
      display.setCursor(0, 12);   display.print("Homed! Measure Top->Bed");
      display.setCursor(0, 24);   display.print("Enter total distance:");
      display.setTextSize(2);
      display.setCursor(15, 40);  display.print(settings.maxPhysicalHeight, 1); display.print(" mm");
      break;

    case CALIBRATE_LENS_MENU:
      display.setCursor(0, 12);   display.print("Edit Lens Focus Dist:");
      for(int i = 0; i < TOTAL_LENSES; i++) {
        int yPos = 21 + (i * 6);
        if (yPos > 57) break;
        display.setCursor(2, yPos);
        if(i == calibrateLensSelectIndex) {
          if(editingLensValue)    display.print("# ");
          else                    display.print("> ");
        } else {
          display.print("  ");
        }
        display.print(lensNames[i]); display.print(": ");
        display.print(settings.lensFocusDistances[i], 1); display.print("mm");
      }
      display.setCursor(2, 57);
      if(calibrateLensSelectIndex == TOTAL_LENSES) display.print("> < Back");
      else                                         display.print("  < Back");
      break;

    case COLOR_OFFSET_MENU:
      display.setCursor(0, 12);   display.print("Edit Color Offsets:");
      for(int i = 1; i < TOTAL_COLORS; i++) {
        int yPos = 23 + ((i - 1) * 8);
        display.setCursor(2, yPos);
        if(i == calibrateColorSelectIndex) {
          if(editingColorValue)   display.print("# ");
          else                    display.print("> ");
        } else {
          display.print("  ");
        }
        display.print(colorNames[i]); display.print(": ");
        if(settings.colorOffsets[i] >= 0) display.print("+");
        display.print(settings.colorOffsets[i], 2); display.print("mm");
      }
      display.setCursor(2, 55);
      if(calibrateColorSelectIndex == TOTAL_COLORS) display.print("> < Back");
      else                                          display.print("  < Back");
      break;

    default:
      break;
  }
  display.display();
}
/*--------------------------------------------------------------------------------------------------------*/


void moveToCalculatedFocus() {
  if (!settings.isCalibrated) return; 
  
  float baseFocus = settings.lensFocusDistances[settings.selectedLens];
  float targetMM = baseFocus + settings.materialHeight - settings.colorOffsets[settings.selectedColorIndex];
  
  if (targetMM > settings.maxPhysicalHeight) targetMM = settings.maxPhysicalHeight;
  if (targetMM < 0) targetMM = 0;

  float mmFromTop = settings.maxPhysicalHeight - targetMM;
  long targetSteps = mmFromTop * STEPS_PER_MM; 

  stepper.moveTo(targetSteps);
}
void executeHome(bool silent) {
  if(!silent) {
    display.clearDisplay();
    display.setCursor(10, 25);
    display.setTextSize(1);
    display.print("Homing z-axis...");
    display.display();
  }

  stepper.setMaxSpeed(1500);

  // 1. KØR HURTIGT MOD TOP
  stepper.setSpeed(-1200);  
  unsigned long endstopActiveTime = 0;
  bool endstopConfirmed = false;

  while (!endstopConfirmed) {
    stepper.runSpeed(); 
    if (digitalRead(ENDSTOP_TOP) == LOW) {
      if (endstopActiveTime == 0) endstopActiveTime = millis();
      else if (millis() - endstopActiveTime > 20) endstopConfirmed = true;
    } else {
      endstopActiveTime = 0;
    }
  }
  
  stepper.setSpeed(0);
  delay(300);
  
  // 2. BAK UD
  stepper.setSpeed(400); 
  unsigned long endstopReleasedTime = 0;
  bool releasedConfirmed = false;

  while (!releasedConfirmed) {
    stepper.runSpeed();
    if (digitalRead(ENDSTOP_TOP) == HIGH) {
      if (endstopReleasedTime == 0) endstopReleasedTime = millis();
      else if (millis() - endstopReleasedTime > 20) releasedConfirmed = true;
    } else {
      endstopReleasedTime = 0;
    }
  }
  
  long startSteps = stepper.currentPosition();
  stepper.setSpeed(600);
  while (abs(stepper.currentPosition() - startSteps) < 3200) {
    stepper.runSpeed();
  }
  delay(300); 

  // 3. KØR LANGSOMT MOD TOP IGEN
  stepper.setSpeed(-150); 
  endstopActiveTime = 0;
  endstopConfirmed = false;

  while (!endstopConfirmed) {
    stepper.runSpeed(); 
    if (digitalRead(ENDSTOP_TOP) == LOW) {
      if (endstopActiveTime == 0) endstopActiveTime = millis();
      else if (millis() - endstopActiveTime > 25) endstopConfirmed = true;
    } else {
      endstopActiveTime = 0;
    }
  }
  
  stepper.setSpeed(0);
  delay(100);

  stepper.setCurrentPosition(0); 
  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1500);
}

void moveWithWaitMessage() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(35, 15);
  display.print("WAIT");
  display.setTextSize(1);
  display.setCursor(15, 45);
  display.print("Changing focus...");
  display.display();

  moveToCalculatedFocus();
  hardware_stepper_run_block();
  saveSettings(); 
}

void hardware_stepper_run_block() {
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
}

/*--------------------------------------------------------------------------------------------------------------------*/
void handleButtons() {
  if (digitalRead(ENCODER_SW) == LOW) {
    if (millis() - lastButtonPress > 300) { // Debounce tid på 300ms
      lastButtonPress = millis();

      switch(currentState) {
        // --- WIZARD TILSTANDE ---
        case WIZARD_WELCOME:
          executeHome();
          currentState = WIZARD_HEIGHT;
          break;

        case WIZARD_HOMING:
          // Vent på at homing bliver færdig
          break;

        case WIZARD_HEIGHT:
          wizardLensIndex = 0;
          currentState = WIZARD_LENSES;
          break;

        case WIZARD_LENSES:
          wizardLensIndex++;
          if (wizardLensIndex >= TOTAL_LENSES) {
            settings.isCalibrated = true;
            saveSettings();
            currentState = STATUS_SCREEN;
            moveToCalculatedFocus();
          }
          break;

        // --- NORMAL NAVIGERING ---
        case STATUS_SCREEN:
          currentState = MAIN_MENU;
          menuIndex = 0;
          break;

        case MAIN_MENU:
          if (menuIndex == 0) { currentState = MAT_HEIGHT_MENU; }
          else if (menuIndex == 1) { currentState = ENGRAVE_3D_SELECT; engraveMenuIndex = 0; }
          else if (menuIndex == 2) { currentState = COLOR_MENU; menuIndex = settings.selectedColorIndex; }
          else if (menuIndex == 3) { currentState = LENS_MENU; menuIndex = settings.selectedLens; }
          else if (menuIndex == 4) { executeHome(); moveToCalculatedFocus(); currentState = STATUS_SCREEN; }
          else if (menuIndex == 5) { currentState = SETUP_MENU; setupMenuIndex = 0; }
          break;

        // --- SETUP MENU ---
        case SETUP_MENU:
          if (setupMenuIndex == 0) {
            executeHome(); 
            currentState = CALIBRATE_HEIGHT_MENU;
          } else if (setupMenuIndex == 1) {
            currentState = CALIBRATE_LENS_MENU;
            calibrateLensSelectIndex = 0;
            editingLensValue = false;
          } else if (setupMenuIndex == 2) {
            currentState = COLOR_OFFSET_MENU;
            calibrateColorSelectIndex = 0;
            editingColorValue = false;
          } else if (setupMenuIndex == 3) { 
            // Run Setup Wizard
            currentState = WIZARD_WELCOME;
          } else if (setupMenuIndex == 4) { 
            // Back / Exit til Hovedmenuen
            currentState = MAIN_MENU;
            menuIndex = 0;
          }
          break;

        case CALIBRATE_HEIGHT_MENU:
          saveSettings();
          currentState = SETUP_MENU;
          break;

        case CALIBRATE_LENS_MENU:
          if (calibrateLensSelectIndex == TOTAL_LENSES) {
            currentState = SETUP_MENU;
          } else {
            if (!editingLensValue) {
              editingLensValue = true;
            } else {
              editingLensValue = false;
              saveSettings();
            }
          }
          break;

        case COLOR_OFFSET_MENU:
          if (calibrateColorSelectIndex == TOTAL_COLORS) {
            currentState = SETUP_MENU;
          } else {
            if (!editingColorValue) {
              editingColorValue = true;
            } else {
              editingColorValue = false;
              saveSettings();
            }
          }
          break;

        case LENS_MENU:
          moveWithWaitMessage(); 
          currentState = STATUS_SCREEN;
          break;

        case ENGRAVE_3D_SELECT:
          if (engraveMenuIndex == 0) {
            currentState = ENGRAVE_3D_MENU; // Gå til Manuel
          } else {
            currentState = ENGRAVE_3D_AUTO; // Gå til Auto
            engraveAutoSetupStep = 0; // Sørg for vi altid starter på "Minutter"
          }
          break;


        // --- 3D AUTO ENGRAVING ---
        case ENGRAVE_3D_AUTO:
          if (engraveAutoSetupStep < 3) {
            // Skift mellem Min -> Sek -> Dybde -> Klar
            engraveAutoSetupStep++; 
          } else if (engraveAutoSetupStep == 3 || engraveAutoSetupStep == 6) {
            // Hvis vi afbryder ved "Klar" (3) eller trykker OK ved "Færdig" (6)
            engraveAutoSetupStep = 0; 
            currentState = STATUS_SCREEN; 
          }
          break;
        case MAT_HEIGHT_MENU:
        case ENGRAVE_3D_MENU:
        case COLOR_MENU:
          saveSettings(); 
          currentState = STATUS_SCREEN;
          break;
      }

      updateDisplay(); // Opdater skærmen med det samme efter tilstandsskift
    }
  }
}

/*-----------------------------------------------------------------------------------------------------*/
