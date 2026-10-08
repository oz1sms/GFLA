/*
* Galvo Fiber Laser Autofocus Control System Ver. 8 (Open-Source Edition)
* Hardware ESP32-WROOM-DA, OLED SSD1309 display i2c (128x64), EC11 Encoder, Endstop
* MKS TMC2160-OC Stepper driver
* 5V 3.3V DC Logic Level Converter Bi-Directional Board Module
* Nema 23 1.2 N.m.
* Quick-buttons: 2 on-off-on momentary switch (+10/10 , +1/-1)  
* Footpedal Button on-off momentary switch
* Relay module 5V to activate Lightburn footpedal switch 
* defined in Lightburn (Start Marking) to start Auto 3D engraving
* 
* IDE: Arduino IDE
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <DIYables_OLED_SSD1309.h>
#include <AccelStepper.h>
#include <EEPROM.h>

// Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define I2C_SDA 21
#define I2C_SCL 22

DIYables_OLED_SSD1309 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// EEPROM Size for storing persistent configuration parameters
#define EEPROM_SIZE 256

// Pin definitions
#define ENCODER_CLK 32
#define ENCODER_DT 14
#define ENCODER_SW 13

#define BTN_PLUS_10 16
#define BTN_MINUS_10 17
#define BTN_PLUS_1 18
#define BTN_MINUS_1 19

#define ENDSTOP_TOP 33

#define STEP_PIN 25
#define DIR_PIN 26
#define EN_PIN 27

int confirmSaveIndex = 0;

unsigned long lastInteractionTime = 0;
const unsigned long SCREENSAVER_TIMEOUT = 300000; // 5 minutter i millisekunder (5 * 60 * 1000)
bool screensaverActive = false;

// --- ESP32 Foot pedal and Relay setup ---
const int buttonPin = 4; 
const int relayPin = 23; 

// --- 3D Auto Engraving Variables ---
int engraveAutoMinutes = 0;
int engraveAutoSeconds = 0;
float engraveAutoDepth = 0.00;
// Setup state tracking for auto engraving flow: 0=Minutes, 1=Seconds, 2=Depth, 3=Wait for pedal, 4=Running, 5=Finished
int engraveAutoSetupStep = 0; 
unsigned long engraveStartTime = 0;
long engraveStartPos = 0;

// Stepper motor configuration (4mm per revolution @ 16 microsteps = 3200 steps)
const float STEPS_PER_MM = 800.0;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Total configurations - Expanded to support up to 6 lenses
const int TOTAL_LENSES = 3; // Max capacity is 6 lenses

// Names of the default lenses
const char* lensNames[TOTAL_LENSES] = { "70x70", "110x110", "150x150" };

const int TOTAL_COLORS = 5;
// Names of the 5 color profiles (Now includes an "Off" state)
const char* colorNames[TOTAL_COLORS] = { "Off", "1", "2", "3", "4" };

// Menu system structures for UI navigation
enum State { 
STATUS_SCREEN, MAIN_MENU, MAT_HEIGHT_MENU, LENS_MENU, CALIBRATE_LENS_MENU, ENGRAVE_3D_AUTO, COLOR_MENU, 
SETUP_MENU, CALIBRATE_HEIGHT_MENU, EDIT_LENS_MENU, COLOR_OFFSET_MENU, 
WIZARD_WELCOME, WIZARD_HOMING, WIZARD_HEIGHT, WIZARD_LENSES, CONFIRM_SAVE_FOCUS
};

State currentState = STATUS_SCREEN;
int engraveMenuIndex = 0; // 0 = Manual mode, 1 = Auto mode

// Global variables structure to be saved to EEPROM persistently
struct Settings {
 bool isCalibrated; // True if the first-time setup wizard has been completed
 int selectedLens; // Selected lens index (0 to 5)
 float materialHeight; // Current material thickness in mm
 float colorOffsets[TOTAL_COLORS]; // 4 different color focus offsets in mm
 int selectedColorIndex; // Active color profile (0-3)
 long currentPositionSteps; // Stores the physical Z-axis position in steps
 float maxPhysicalHeight; // Calibrated max height from the top endstop to the bed
 float lensFocusDistances[TOTAL_LENSES]; // Pre-configured focus distances for the 6 lenses
} settings;

const int EEPROM_ADDR = 0;

// Encoder variables for reading rotation and button states
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
const int SETUP_MENU_COUNT = 6; 
const char* setupMenuItems[SETUP_MENU_COUNT] = {
 "1. Calibrate Max H",
 "2. Edit Lens Focus",
 "3. Edit Color Offsets",
 "4. Run Setup Wizard",
 "5. Calibrate Focus",
 "6. Back / Exit"
};

// Variables for the color menu
int calibrateLensSelectIndex = 0; 
int calibrateColorSelectIndex = 1; // Added index mapping
bool editingLensValue = false;
bool editingColorValue = false; // Added state
int wizardLensIndex = 0; 

// Function prototypes
void updateDisplay();
void saveSettings();
void moveToCalculatedFocus();
void executeHome(bool silent = false);
void handleEncoder();
void handleButtons();
void handleQuickButtons();
void moveWithWaitMessage();
void hardware_stepper_run_block();

// Interrupt routine for the foot pedal
void IRAM_ATTR handleFootPedal() {
 delayMicroseconds(10000); 

 bool isPressed = (digitalRead(buttonPin) == LOW);
 
 if (isPressed) {
 digitalWrite(relayPin, LOW); 
 } else {
 digitalWrite(relayPin, HIGH); 
 }
}

void handleScreensaver() {
  if (millis() - lastInteractionTime > SCREENSAVER_TIMEOUT) {
    if (!screensaverActive) {
      display.clearDisplay();
      display.display();
      display.ssd1309_command(0x81); // Kontrast kommando
      display.ssd1309_command(0x00); // Dæmp skærmen (0-255)
      screensaverActive = true;
    }
  } else {
    if (screensaverActive) {
      display.ssd1309_command(0x81); 
      display.ssd1309_command(0xCF); 
      screensaverActive = false;
      updateDisplay(); 
    }
  }
}

void setup() {
 Serial.begin(115200);
 Serial.println("System starting...");

 // Pin configurations
 pinMode(ENCODER_CLK, INPUT_PULLUP);
 pinMode(ENCODER_DT, INPUT_PULLUP);
 pinMode(ENCODER_SW, INPUT_PULLUP);
 pinMode(ENDSTOP_TOP, INPUT_PULLUP);
 pinMode(EN_PIN, OUTPUT);
 
 pinMode(BTN_PLUS_10, INPUT_PULLUP);
 pinMode(BTN_MINUS_10, INPUT_PULLUP);
 pinMode(BTN_PLUS_1, INPUT_PULLUP);
 pinMode(BTN_MINUS_1, INPUT_PULLUP);

 digitalWrite(EN_PIN, LOW); // Enable the TMC2160 stepper driver
 lastClkVal = digitalRead(ENCODER_CLK);

 stepper.setMaxSpeed(3000);
 stepper.setAcceleration(1500);
 
 // 1. Set up the relay and turn it OFF immediately
 pinMode(relayPin, OUTPUT);
 digitalWrite(relayPin, HIGH); 

 // 2. Configure the foot pedal interrupt
 pinMode(buttonPin, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(buttonPin), handleFootPedal, CHANGE);
 
 // Initialize OLED display
 if(!display.begin(SSD1309_SWITCHCAPVCC, 0x3C)) { 
 Serial.println(F("SSD1309 allocation failed"));
 for(;;); 
 }
 display.clearDisplay();
 display.setTextColor(SSD1309_WHITE);

 // Initialize EEPROM
 if (!EEPROM.begin(EEPROM_SIZE)) {
 Serial.println("EEPROM error!");
 } else {
 EEPROM.get(EEPROM_ADDR, settings);

 if (settings.isCalibrated != true || isnan(settings.materialHeight) || settings.selectedLens < 0 || settings.selectedLens >= TOTAL_LENSES) {
 Serial.println("No valid data found. Starting First-Time Setup Wizard...");
 
 settings.isCalibrated = false;
 settings.selectedLens = 0;
 settings.materialHeight = 0.0;
 settings.selectedColorIndex = 0;
 settings.currentPositionSteps = 0;
 settings.maxPhysicalHeight = 515.0; 
 
 settings.lensFocusDistances[0] = 100.0; // 70x70
 settings.lensFocusDistances[1] = 160.0; // 110x110
 settings.lensFocusDistances[2] = 210.0; // 150x150
 settings.lensFocusDistances[3] = 254.0; // 175x175
 settings.lensFocusDistances[4] = 290.0; // 200x200
 settings.lensFocusDistances[5] = 420.0; // 300x300

 for(int i = 0; i < TOTAL_COLORS; i++) {
 settings.colorOffsets[i] = 0.0;
 }
 
 currentState = WIZARD_WELCOME; 
 } else {
 stepper.setCurrentPosition(settings.currentPositionSteps);
 currentState = STATUS_SCREEN;
 }
 }
 
 updateDisplay();
 Serial.println("System ready!");
}

void loop() {

handleScreensaver(); 

 // --- 3D Auto Engraving Logic Execution ---
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
 engraveAutoSetupStep = 4; // Switch to "Running" state
 updateDisplay(); 
 } else {
 engraveAutoSetupStep = 6; // Invalid time/depth - skip to finished state
 updateDisplay();
 }
 }
 } 
 else if (engraveAutoSetupStep == 4) {
  unsigned long totalTimeMs = (engraveAutoMinutes * 60UL + engraveAutoSeconds) * 1000UL;
  unsigned long elapsed = millis() - engraveStartTime;
 
 if (elapsed <= totalTimeMs) {
 stepper.runSpeed(); 
 } else {
 stepper.setMaxSpeed(3000); 
 stepper.moveTo(engraveStartPos); 
 engraveAutoSetupStep = 5; 
 updateDisplay();
 }
 }
 else if (engraveAutoSetupStep == 5) {
 if (stepper.distanceToGo() != 0) {
 stepper.run(); 
 } else {
 engraveAutoSetupStep = 6; 
 updateDisplay();
 }
 } 
 else {
 stepper.run();
 }
 } else {
 stepper.run(); 
 }

 handleEncoder();
 handleButtons();
 handleQuickButtons();
}

// Saves current configuration and position persistently
void saveSettings() {
 settings.currentPositionSteps = stepper.currentPosition();
 EEPROM.put(EEPROM_ADDR, settings);
 EEPROM.commit();
 Serial.println("Settings autosaved to EEPROM.");
}

// Handles rotary encoder turns based on the current active screen
void handleEncoder() { 
 int clkVal = digitalRead(ENCODER_CLK); 
 if (clkVal != lastClkVal && clkVal == LOW) { 
 int direction = (digitalRead(ENCODER_DT) != clkVal) ? 1 : -1; 

 lastInteractionTime = millis(); 

 switch(currentState) { 
 case STATUS_SCREEN:
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

 case CALIBRATE_LENS_MENU: { 
  long steps = direction * (0.01 * STEPS_PER_MM); 
 stepper.move(steps); 
 break; 
 } 
 
 case CONFIRM_SAVE_FOCUS:
 confirmSaveIndex += direction;
 if (confirmSaveIndex < 0) confirmSaveIndex = 1;
 if (confirmSaveIndex > 1) confirmSaveIndex = 0;
 break;

 case ENGRAVE_3D_AUTO:
 if (engraveAutoSetupStep == 0) {
 engraveAutoMinutes = constrain(engraveAutoMinutes + direction, 0, 59);
 } else if (engraveAutoSetupStep == 1) {
 engraveAutoSeconds = constrain(engraveAutoSeconds + direction, 0, 59);
 } else if (engraveAutoSetupStep == 2) {
 engraveAutoDepth = constrain(engraveAutoDepth + (direction * 0.01), 0.0, 5.0);
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

 case EDIT_LENS_MENU: 
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

void handleQuickButtons() {
 if (currentState != WIZARD_HOMING && 
 currentState != WIZARD_HEIGHT && 
 currentState != WIZARD_LENSES && 
 currentState != MAT_HEIGHT_MENU &&
 currentState != CALIBRATE_HEIGHT_MENU && 
 currentState != EDIT_LENS_MENU) {
 return; 
 }

 float adjustment = 0.0;
 if (digitalRead(BTN_PLUS_10) == LOW) adjustment = 10.0; 
 else if (digitalRead(BTN_MINUS_10) == LOW) adjustment = -10.0;
 else if (digitalRead(BTN_PLUS_1) == LOW) adjustment = 1.0;
 else if (digitalRead(BTN_MINUS_1) == LOW) adjustment = -1.0;

 if (adjustment != 0.0) {
  if (millis() - lastButtonPress > 200) { 
 lastButtonPress = millis();

 switch(currentState) {
 case WIZARD_HOMING:
 break;

 case MAT_HEIGHT_MENU:
 settings.materialHeight = constrain(settings.materialHeight + adjustment, 0.0, settings.maxPhysicalHeight);
 moveToCalculatedFocus(); 
 break;

 case WIZARD_HEIGHT:
 case CALIBRATE_HEIGHT_MENU:
 settings.maxPhysicalHeight = constrain(settings.maxPhysicalHeight + adjustment, 50.0, 600.0);
 break;

 case WIZARD_LENSES:
 settings.lensFocusDistances[wizardLensIndex] = constrain(settings.lensFocusDistances[wizardLensIndex] + adjustment, 0.0, 530.0);
 break;

 case EDIT_LENS_MENU:
 if (editingLensValue && calibrateLensSelectIndex < TOTAL_LENSES) {
 settings.lensFocusDistances[calibrateLensSelectIndex] = constrain(settings.lensFocusDistances[calibrateLensSelectIndex] + adjustment, 0.0, 530.0);
 }
 break;
 }
 updateDisplay();
 }
 }
}

void updateDisplay() {
 display.clearDisplay();
 display.setTextSize(1);
 display.setTextColor(SSD1309_WHITE);
 
 if (currentState == WIZARD_WELCOME) {
 display.setCursor(15, 5); display.print("GALVO FOCUS OS");
 display.drawFastHLine(0, 15, 128, SSD1309_WHITE);
 display.setCursor(0, 25); display.print("No calibration found");
 display.setCursor(0, 40); display.print("Press Encoder to");
 display.setCursor(0, 50); display.print("start Setup Wizard");
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
 display.setCursor(0, 0); display.print("WIZARD STEP 2/3");
 display.drawFastHLine(0, 10, 128, SSD1309_WHITE);
 display.setCursor(0, 15); display.print("Measure Top -> Bed:");
 display.setTextSize(2);
 display.setCursor(15, 32); display.print(settings.maxPhysicalHeight, 1); display.print(" mm");
 display.setTextSize(1);
 display.setCursor(0, 55); display.print("Turn: Adj. | Press: Next");
 display.display();
 return;
 }
 if (currentState == WIZARD_LENSES) {
 display.setCursor(0, 0); display.print("WIZARD STEP 3/3");
 display.drawFastHLine(0, 10, 128, SSD1309_WHITE);
 display.setCursor(0, 15); display.print("Enter Focus for Lens");
 display.setCursor(0, 26); display.print(wizardLensIndex + 1); display.print("/"); display.print(TOTAL_LENSES);
 display.print(" ["); display.print(lensNames[wizardLensIndex]); display.print("]");
 
 display.setTextSize(2);
 display.setCursor(25, 40); display.print(settings.lensFocusDistances[wizardLensIndex], 1); display.print(" mm");
 
 display.setTextSize(1);
 display.display();
 return;
 }

if (currentState != SETUP_MENU && currentState != ENGRAVE_3D_AUTO && currentState != CONFIRM_SAVE_FOCUS) {
  display.setCursor(0, 0);
  display.print("LENS:");
  display.print(lensNames[settings.selectedLens]);
  
  if (settings.selectedColorIndex != 0) {
    display.print(" COL:"); 
    display.print(colorNames[settings.selectedColorIndex]); 
  }
  
  display.drawFastHLine(0, 10, 128, SSD1309_WHITE);
} 

 switch(currentState) {
 case STATUS_SCREEN: {
 float currentPosFromTopMM = (float)stepper.currentPosition() / STEPS_PER_MM;
  float currentPosMM = settings.maxPhysicalHeight - currentPosFromTopMM;
 
 display.setCursor(0, 15); display.print("Focus Dist:"); 
 display.setCursor(72, 15); display.print(currentPosMM, 2); display.print(" mm");
 
 display.setCursor(0, 28); display.print("Material :");
 display.setCursor(72, 28); display.print(settings.materialHeight, 2); display.print(" mm");
 
 display.drawFastHLine(0, 42, 128, SSD1309_WHITE);
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
 display.setCursor(10, 20); display.print("Material Height:");
 display.setTextSize(2);
 display.setCursor(20, 35); display.print(settings.materialHeight, 2); display.print(" mm");
 break;

 case LENS_MENU:
 display.setCursor(10, 12); display.print("Select Lens:");
 for(int i = 0; i < TOTAL_LENSES; i++) {
 int yPos = 22 + (i * 7);
 if (yPos > 57) break;
 display.setCursor(5, yPos);
 if(i == settings.selectedLens) display.print("> ");
 else display.print(" ");
 display.print(lensNames[i]);
 display.print(" (F"); display.print(settings.lensFocusDistances[i], 1); display.print(")");
 }
 break;

 case COLOR_MENU:
 display.setCursor(1, 15); display.print("Select Color Profile:");
 display.setTextSize(2);
 display.setCursor(40, 35);
 
 if (settings.selectedColorIndex == 0) {
 display.print("OFF");
 } else {
 display.print("# "); 
 display.print(settings.selectedColorIndex); 
 }
 break;

 case ENGRAVE_3D_AUTO: {
 display.setCursor(0, 0);
 display.print("3D AUTO ENGRAVING");
 display.drawFastHLine(0, 10, 128, SSD1309_WHITE);
 
 if (engraveAutoSetupStep == 0) display.setTextColor(SSD1309_BLACK, SSD1309_WHITE);
 else display.setTextColor(SSD1309_WHITE);
 display.setCursor(0, 15);
 display.print("Min: "); display.print(engraveAutoMinutes);
 
 if (engraveAutoSetupStep == 1) display.setTextColor(SSD1309_BLACK, SSD1309_WHITE);
 else display.setTextColor(SSD1309_WHITE);
 display.setCursor(64, 15);
 display.print("Sec: "); display.print(engraveAutoSeconds); 
 
 if (engraveAutoSetupStep == 2) display.setTextColor(SSD1309_BLACK, SSD1309_WHITE);
 else display.setTextColor(SSD1309_WHITE);
 display.setCursor(0, 28);
 display.print("Depth: "); display.print(engraveAutoDepth, 2); display.print(" mm"); 

 display.setTextColor(SSD1309_WHITE);
 display.drawFastHLine(0, 42, 128, SSD1309_WHITE);
 display.setCursor(0, 46);
 
 if (engraveAutoSetupStep < 3) {
 display.print("Turn to change"); 
 display.setCursor(0, 56); display.print("Press Encoder: Next"); 
 } else if (engraveAutoSetupStep == 3) {
 display.print("Prepare Lightburn"); 
 display.setCursor(0, 56); display.print("Red GO / Press EXIT"); 
 } else if (engraveAutoSetupStep == 4) {
 display.print("DIVING DOWN..."); 
 } else if (engraveAutoSetupStep == 5) {
 display.print("RETURNING..."); 
 } else if (engraveAutoSetupStep == 6) {
 display.print("FINISHED!"); 
 display.setCursor(0, 56); display.print("Press for menu"); 
 }
 break;
 }

 case CALIBRATE_LENS_MENU: {
 float currentPosFromTopMM = (float)stepper.currentPosition() / STEPS_PER_MM;
  float currentPosMM = settings.maxPhysicalHeight - currentPosFromTopMM;

 display.setCursor(87, 0); display.print("Calib.");
 
 display.setCursor(10, 18); display.print("Focus: ");
 display.print(currentPosMM, 2); 
 display.print(" mm");
 
 display.setCursor(10, 36); display.print("Turn: +/- 0.01mm");
 display.setCursor(10, 52); display.print("Press SW to save");
 break;
 }
 
 case CONFIRM_SAVE_FOCUS: {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("Gem ny fokus for:"));
  display.println(lensNames[settings.selectedLens]);
  display.println("");
  
  if (confirmSaveIndex == 0) {
    display.setTextColor(SSD1309_BLACK, SSD1309_WHITE);
    display.println(F("> JA, GEM   "));
    display.setTextColor(SSD1309_WHITE, SSD1309_BLACK);
    display.println(F("  NEJ, ANNULLER"));
  } else {
    display.setTextColor(SSD1309_WHITE, SSD1309_BLACK);
    display.println(F("  JA, GEM   "));
    display.setTextColor(SSD1309_BLACK, SSD1309_WHITE);
    display.println(F("> NEJ, ANNULLER"));
  }
  break;
 }

 case SETUP_MENU: {
 display.setCursor(10, 0); 
 display.print("--- SETUP MENU ---");
 display.drawFastHLine(0, 10, 128, SSD1309_WHITE);

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
 display.setCursor(0, 12); display.print("Homed! Measure Top->Bed");
 display.setCursor(0, 24); display.print("Enter total distance:");
 display.setTextSize(2);
 display.setCursor(15, 40); display.print(settings.maxPhysicalHeight, 1); display.print(" mm");
 break;

 case EDIT_LENS_MENU:
 display.setCursor(0, 12); display.print("Edit Lens Focus Dist:");
 for(int i = 0; i < TOTAL_LENSES; i++) {
 int yPos = 21 + (i * 6);
 if (yPos > 57) break;
 display.setCursor(2, yPos);
 if(i == calibrateLensSelectIndex) {
 if(editingLensValue) display.print("# ");
 else display.print("> ");
 } else {
 display.print(" ");
 }
 display.print(lensNames[i]); display.print(": ");
 display.print(settings.lensFocusDistances[i], 1); display.print("mm");
 }
 display.setCursor(2, 57);
 if(calibrateLensSelectIndex == TOTAL_LENSES) display.print("> < Back");
 else display.print(" < Back");
 break;

 case COLOR_OFFSET_MENU:
 display.setCursor(0, 12); display.print("Edit Color Offsets:");
 for(int i = 1; i < TOTAL_COLORS; i++) {
  int yPos = 23 + ((i - 1) * 8);
 display.setCursor(2, yPos);
 if(i == calibrateColorSelectIndex) {
 if(editingColorValue) display.print("# ");
 else display.print("> ");
 } else {
 display.print(" ");
 }
 display.print(colorNames[i]); display.print(": ");
 if(settings.colorOffsets[i] >= 0) display.print("+");
 display.print(settings.colorOffsets[i], 2); display.print("mm");
 }
 display.setCursor(2, 55);
 if(calibrateColorSelectIndex == TOTAL_COLORS) display.print("> < Back");
 else display.print(" < Back");
 break;

 default:
 break;
 }
 display.display();
}

void moveToCalculatedFocus() {
 if (!settings.isCalibrated) return; 

 float baseFocus = settings.lensFocusDistances[settings.selectedLens];
  float targetMM = baseFocus + settings.materialHeight + settings.colorOffsets[settings.selectedColorIndex];
 
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

 stepper.setMaxSpeed(4000); // 1500

 // 1. DRIVE FAST TOWARDS TOP
 stepper.setSpeed(-4000); // -1200
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
 
 // 2. BACK OUT
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

 // 3. DRIVE SLOWLY TOWARDS TOP AGAIN
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

void handleButtons() {
 if (digitalRead(ENCODER_SW) == LOW) {
  if (millis() - lastButtonPress > 300) { 
 lastButtonPress = millis();

 switch(currentState) {
 case WIZARD_WELCOME:
 executeHome();
 currentState = WIZARD_HEIGHT;
 break;

 case WIZARD_HOMING:
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
 moveWithWaitMessage();
 //moveToCalculatedFocus();
 }
 break;

 case STATUS_SCREEN:
 currentState = MAIN_MENU;
 menuIndex = 0;
 break;

 case MAIN_MENU:
 if (menuIndex == 0) { currentState = MAT_HEIGHT_MENU; }
 else if (menuIndex == 1) { 
   currentState = ENGRAVE_3D_AUTO; 
   engraveAutoSetupStep = 0;       
 }
 else if (menuIndex == 2) { currentState = COLOR_MENU; menuIndex = settings.selectedColorIndex; }
 else if (menuIndex == 3) { currentState = LENS_MENU; menuIndex = settings.selectedLens; }
 else if (menuIndex == 4) { executeHome(); moveWithWaitMessage(); currentState = STATUS_SCREEN; }
 else if (menuIndex == 5) { currentState = SETUP_MENU; setupMenuIndex = 0; }
 break;

 case SETUP_MENU:
 if (setupMenuIndex == 0) {
   executeHome(); 
   currentState = CALIBRATE_HEIGHT_MENU;
 } else if (setupMenuIndex == 1) {
   currentState = EDIT_LENS_MENU;
   calibrateLensSelectIndex = 0;
   editingLensValue = false;
 } else if (setupMenuIndex == 2) {
   currentState = COLOR_OFFSET_MENU;
   calibrateColorSelectIndex = 0;
   editingColorValue = false;
 } else if (setupMenuIndex == 3) { 
   currentState = WIZARD_WELCOME;
 } else if (setupMenuIndex == 4) { 
   currentState = CALIBRATE_LENS_MENU; 
   engraveMenuIndex = 0; 
 } else if (setupMenuIndex == 5) { 
   currentState = MAIN_MENU;
   menuIndex = 0;
 }
 break;

 case CALIBRATE_HEIGHT_MENU:
 saveSettings();
 currentState = SETUP_MENU;
 break;

 case EDIT_LENS_MENU:
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

 case ENGRAVE_3D_AUTO:
 if (engraveAutoSetupStep < 3) {
 engraveAutoSetupStep++; 
 } else if (engraveAutoSetupStep == 3 || engraveAutoSetupStep == 6) {
 engraveAutoSetupStep = 0; 
 currentState = STATUS_SCREEN; 
 }
 break;
 
 // ---- Her er ændringen for CALIBRATE_LENS_MENU ----
 case CALIBRATE_LENS_MENU:
 confirmSaveIndex = 0; // Standard-valg er "Ja"
 currentState = CONFIRM_SAVE_FOCUS;
 break;

 // ---- Her håndterer vi Ja/Nej valget ----
 case CONFIRM_SAVE_FOCUS:
  if (confirmSaveIndex == 0) {
    // Brugeren valgte "JA"
    float newFocusDistance = settings.maxPhysicalHeight - (stepper.currentPosition() / STEPS_PER_MM);
    settings.lensFocusDistances[settings.selectedLens] = newFocusDistance;
    saveSettings(); 
    Serial.println("Ny fokusværdi gemt!");
  } else {
    // Brugeren valgte "NEJ"
    Serial.println("Ændring annulleret.");
  }
  currentState = SETUP_MENU;
  break;

 case MAT_HEIGHT_MENU:
 case COLOR_MENU:
 saveSettings(); 
 currentState = STATUS_SCREEN;
 break;
 }

 updateDisplay(); 
 }
 }
}