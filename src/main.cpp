#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// =========== PIN DEFINITIONS ===========
// DISPLAY SH1107 (I2C)
#define I2C_SDA 21
#define I2C_SCL 22
#define OLED_ADDRESS 0x3C // o 0x3D

// STEPPER 1 (X)
#define STEP_PIN_X 14
#define DIR_PIN_X 27
#define ENABLE_PIN_X 26

// STEPPER 2 (Y)
#define STEP_PIN_Y 25
#define DIR_PIN_Y 33
#define ENABLE_PIN_Y 32

// LIMIT SWITCHES
#define LIMIT_X_MIN 34
#define LIMIT_X_MAX 35
#define LIMIT_Y_MIN 36
#define LIMIT_Y_MAX 39

// PULSANTI (con pull-up esterni da 10k)
#define BUTTON_START 12
#define BUTTON_STOP 13
#define BUTTON_PAUSE 5

// POTENZIOMETRI
#define POTENZIOMETRO_MOTORE1 15
#define POTENZIOMETRO_MOTORE2 4

// RELE'
#define RELE_PIN 3

// =========== STATI SISTEMA ===========
enum SystemState
{
  STATE_OFF,
  STATE_HOMING,
  STATE_READY,
  STATE_RUNNING,
  STATE_PAUSED,
  STATE_EMERGENCY
};
SystemState currentState = STATE_OFF;

// =========== VARIABILI GLOBALI ===========
// Debouncing
unsigned long lastDebounceTimeStart = 0;
unsigned long lastDebounceTimeStop = 0;
unsigned long lastDebounceTimePause = 0;
const unsigned long debounceDelay = 50;

// CON PULL-UP ESTERNI, LO STATO PREMUTO È LOW E NON PREMUTO È HIGH
int buttonStartState = HIGH;
int lastButtonStartState = HIGH;
int buttonStopState = HIGH;
int lastButtonStopState = HIGH;
int buttonPauseState = HIGH;
int lastButtonPauseState = HIGH;

bool startPressed = false;
bool stopPressed = false;
bool pausePressed = false;

// Ciclo coordinato
bool cicloAttivo = false;
bool motore1Completato = false;
bool motore2Completato = false;
unsigned long tempoInizioCiclo = 0;

// Potenziometri
int velocitaMotore1 = 1000;
int velocitaMotore2 = 800;
int ultimaLetturaPot1 = 0;
int ultimaLetturaPot2 = 0;
unsigned long ultimoAggiornamentoPot = 0;
const unsigned long INTERVALLO_POT = 100;
const int VELOCITA_MIN = 100;
const int VELOCITA_MAX = 3000;

// Stato motori e relè
bool motoriAbilitati = false;
bool releAttivo = false;

// Variabili per homing e movimento
bool homingInCorsoX = false;
bool homingInCorsoY = false;
long posizioneTargetY = 0;
bool movimentoYAttivo = false;

// =========== OGGETTI GLOBALI ===========
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);
Adafruit_SH1107 display = Adafruit_SH1107(128, 128, &Wire);

// =========== PROTOTIPI FUNZIONI ===========
void emergencyStop();
void checkButtons();
void aggiornaPotenziometri();
void professionalHoming();
void updateDisplay();
void cicloCoordinato();
void avviaMotore2();
void controllaMotore2();
void pauseCycle();
void resumeCycle();
void attivaRele();
void disattivaRele();
void abilitaMotori();
void disabilitaMotori();
void setupDisplay();
void gestisciHoming();

// =========== SETUP DISPLAY ===========
void setupDisplay()
{
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!display.begin(OLED_ADDRESS, true))
  {
    Serial.println("SH1107 non trovato!");
    while (1)
      ;
  }

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.display();

  Serial.println("Display SH1107 inizializzato");
}

// =========== GESTIONE MOTORI ===========
void abilitaMotori()
{
  if (!motoriAbilitati)
  {
    digitalWrite(ENABLE_PIN_X, LOW);
    digitalWrite(ENABLE_PIN_Y, LOW);
    motoriAbilitati = true;
    Serial.println("Motori abilitati");
  }
}

void disabilitaMotori()
{
  if (motoriAbilitati)
  {
    digitalWrite(ENABLE_PIN_X, HIGH);
    digitalWrite(ENABLE_PIN_Y, HIGH);
    motoriAbilitati = false;
    Serial.println("Motori disabilitati");
  }
}

// =========== GESTIONE RELE ===========
void attivaRele()
{
  if (!releAttivo)
  {
    digitalWrite(RELE_PIN, HIGH);
    releAttivo = true;
    Serial.println("Rele attivato");
  }
}

void disattivaRele()
{
  if (releAttivo)
  {
    digitalWrite(RELE_PIN, LOW);
    releAttivo = false;
    Serial.println("Rele disattivato");
  }
}

// =========== SETUP ===========
void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Display prima di tutto
  setupDisplay();

  display.setTextSize(2);
  display.setCursor(10, 50);
  display.print("AVVIO...");
  display.display();
  delay(1000);

  // Stepper enable
  pinMode(ENABLE_PIN_X, OUTPUT);
  pinMode(ENABLE_PIN_Y, OUTPUT);
  disabilitaMotori();

  // PULSANTI CON PULL-UP ESTERNI - USA INPUT SENZA PULLUP INTERNO
  pinMode(BUTTON_START, INPUT);
  pinMode(BUTTON_STOP, INPUT);
  pinMode(BUTTON_PAUSE, INPUT);

  // Finecorsa
  pinMode(LIMIT_X_MIN, INPUT_PULLUP);
  pinMode(LIMIT_X_MAX, INPUT_PULLUP);
  pinMode(LIMIT_Y_MIN, INPUT_PULLUP);
  pinMode(LIMIT_Y_MAX, INPUT_PULLUP);

  // Relè
  pinMode(RELE_PIN, OUTPUT);
  disattivaRele();

  // AccelStepper configuration
  stepperX.setMaxSpeed(3000.0);
  stepperX.setAcceleration(2000.0);
  stepperX.setSpeed(velocitaMotore1);
  
  stepperY.setMaxSpeed(3000.0);
  stepperY.setAcceleration(2000.0);
  stepperY.setSpeed(velocitaMotore2);

  currentState = STATE_OFF;

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(20, 50);
  display.print("PRONTO");
  display.display();

  Serial.println("Sistema pronto - Display SH1107 attivo");
  Serial.println("Configurazione: Pulsanti con pull-up esterni da 10k");
}

// =========== EMERGENCY STOP ===========
void emergencyStop()
{
  stepperX.stop();
  stepperY.stop();
  disabilitaMotori();
  disattivaRele();
  cicloAttivo = false;
  homingInCorsoX = false;
  homingInCorsoY = false;
  movimentoYAttivo = false;
  currentState = STATE_EMERGENCY;
  Serial.println("FERMO EMERGENZA");
}

// =========== GESTIONE PULSANTI (CON PULL-UP ESTERNI) ===========
void checkButtons()
{
  // LEGGI STATO PULSANTI - CON PULL-UP ESTERNI:
  // LOW = PREMUTO, HIGH = NON PREMUTO
  int currentStart = digitalRead(BUTTON_START);
  int currentStop = digitalRead(BUTTON_STOP);
  int currentPause = digitalRead(BUTTON_PAUSE);

  startPressed = false;
  stopPressed = false;
  pausePressed = false;

  // DEBOUNCING START
  if (currentStart != lastButtonStartState)
  {
    lastDebounceTimeStart = millis();
  }
  if ((millis() - lastDebounceTimeStart) > debounceDelay)
  {
    if (currentStart != buttonStartState)
    {
      buttonStartState = currentStart;
      // CON PULL-UP ESTERNI: LOW = PREMUTO
      if (buttonStartState == LOW)
        startPressed = true;
    }
  }
  lastButtonStartState = currentStart;

  // DEBOUNCING STOP
  if (currentStop != lastButtonStopState)
  {
    lastDebounceTimeStop = millis();
  }
  if ((millis() - lastDebounceTimeStop) > debounceDelay)
  {
    if (currentStop != buttonStopState)
    {
      buttonStopState = currentStop;
      // CON PULL-UP ESTERNI: LOW = PREMUTO
      if (buttonStopState == LOW)
        stopPressed = true;
    }
  }
  lastButtonStopState = currentStop;

  // DEBOUNCING PAUSE
  if (currentPause != lastButtonPauseState)
  {
    lastDebounceTimePause = millis();
  }
  if ((millis() - lastDebounceTimePause) > debounceDelay)
  {
    if (currentPause != buttonPauseState)
    {
      buttonPauseState = currentPause;
      // CON PULL-UP ESTERNI: LOW = PREMUTO
      if (buttonPauseState == LOW)
        pausePressed = true;
    }
  }
  lastButtonPauseState = currentPause;

  // DEBUG PULSANTI (opzionale)
  static unsigned long lastDebugButtons = 0;
  if (millis() - lastDebugButtons > 1000)
  {
    Serial.printf("Pulsanti - START:%d STOP:%d PAUSE:%d\n", currentStart, currentStop, currentPause);
    lastDebugButtons = millis();
  }

  if (stopPressed)
  {
    emergencyStop();
    return;
  }

  switch (currentState)
  {
  case STATE_OFF:
    if (startPressed)
    {
      currentState = STATE_HOMING;
      abilitaMotori();
      Serial.println("Pulsante START premuto - Inizio homing");
    }
    break;
  case STATE_READY:
    if (startPressed)
    {
      currentState = STATE_RUNNING;
      abilitaMotori();
      Serial.println("Pulsante START premuto - Inizio ciclo");
    }
    break;
  case STATE_PAUSED:
    if (startPressed)
    {
      resumeCycle();
      abilitaMotori();
      Serial.println("Pulsante START premuto - Ripresa ciclo");
    }
    break;
  case STATE_RUNNING:
    if (pausePressed)
    {
      pauseCycle();
      disabilitaMotori();
      Serial.println("Pulsante PAUSE premuto - Pausa ciclo");
    }
    break;
  case STATE_EMERGENCY:
    if (startPressed)
    {
      currentState = STATE_OFF;
      disabilitaMotori();
      Serial.println("Pulsante START premuto - Reset da emergenza");
    }
    break;
  }
}

// =========== GESTIONE POTENZIOMETRI ===========
void aggiornaPotenziometri()
{
  if (millis() - ultimoAggiornamentoPot > INTERVALLO_POT)
  {
    int letturaPot1 = analogRead(POTENZIOMETRO_MOTORE1);
    int letturaPot2 = analogRead(POTENZIOMETRO_MOTORE2);

    if (abs(letturaPot1 - ultimaLetturaPot1) > 50)
    {
      velocitaMotore1 = map(letturaPot1, 0, 4095, VELOCITA_MIN, VELOCITA_MAX);
      ultimaLetturaPot1 = letturaPot1;
      if (currentState == STATE_READY || currentState == STATE_RUNNING)
      {
        stepperX.setMaxSpeed(velocitaMotore1);
        if (!stepperX.isRunning()) {
          stepperX.setSpeed(velocitaMotore1);
        }
      }
    }

    if (abs(letturaPot2 - ultimaLetturaPot2) > 50)
    {
      velocitaMotore2 = map(letturaPot2, 0, 4095, VELOCITA_MIN, VELOCITA_MAX);
      ultimaLetturaPot2 = letturaPot2;
      if (currentState == STATE_READY || currentState == STATE_RUNNING)
      {
        stepperY.setMaxSpeed(velocitaMotore2);
        if (!stepperY.isRunning()) {
          stepperY.setSpeed(velocitaMotore2);
        }
      }
    }

    ultimoAggiornamentoPot = millis();
  }
}

// =========== HOMING INTELLIGENTE ===========
void gestisciHoming()
{
  static unsigned long homingStartTime = 0;
  static bool homingXCompletato = false;
  static bool homingYCompletato = false;

  if (homingStartTime == 0) {
    homingStartTime = millis();
    homingXCompletato = false;
    homingYCompletato = false;
    
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(10, 50);
    display.print("HOMING...");
    display.display();

    Serial.println("=== HOMING INTELLIGENTE ===");
  }

  // Homing Asse X
  if (!homingXCompletato) {
    if (!homingInCorsoX) {
      // Se già sul finecorsa MIN, si allontana
      if (digitalRead(LIMIT_X_MIN) == LOW) {
        Serial.println("X già su MIN - mi allontano");
        stepperX.moveTo(stepperX.currentPosition() + 10000);
        homingInCorsoX = true;
      } else {
        // Cerca il finecorsa MIN
        stepperX.moveTo(-1000000);
        homingInCorsoX = true;
      }
    } else {
      // Controlla se ha raggiunto il finecorsa
      if (digitalRead(LIMIT_X_MIN) == LOW) {
        stepperX.stop();
        stepperX.setCurrentPosition(0);
        homingInCorsoX = false;
        homingXCompletato = true;
        Serial.println("Homing X completato");
        
        // Muove leggermente via dal finecorsa
        stepperX.moveTo(1000);
      }
    }
  }

  // Homing Asse Y (solo dopo che X è completato)
  if (homingXCompletato && !homingYCompletato) {
    if (!homingInCorsoY) {
      // Se già sul finecorsa MIN, si allontana
      if (digitalRead(LIMIT_Y_MIN) == LOW) {
        Serial.println("Y già su MIN - mi allontano");
        stepperY.moveTo(stepperY.currentPosition() + 10000);
        homingInCorsoY = true;
      } else {
        // Cerca il finecorsa MIN
        stepperY.moveTo(-1000000);
        homingInCorsoY = true;
      }
    } else {
      // Controlla se ha raggiunto il finecorsa
      if (digitalRead(LIMIT_Y_MIN) == LOW) {
        stepperY.stop();
        stepperY.setCurrentPosition(0);
        homingInCorsoY = false;
        homingYCompletato = true;
        Serial.println("Homing Y completato");
        
        // Muove leggermente via dal finecorsa
        stepperY.moveTo(1000);
      }
    }
  }

  // Controlla timeout
  if (millis() - homingStartTime > 30000) {
    emergencyStop();
    Serial.println("Homing timeout");
    return;
  }

  // Se entrambi completati
  if (homingXCompletato && homingYCompletato) {
    // Attende che entrambi i motori completino il movimento di allontanamento
    if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {
      homingStartTime = 0;
      currentState = STATE_READY;
      Serial.println("Homing completato!");
    }
  }
}

// =========== CICLO COORDINATO ===========
void cicloCoordinato()
{
  if (!cicloAttivo)
  {
    // INIZIO CICLO
    cicloAttivo = true;
    motore1Completato = false;
    motore2Completato = false;
    tempoInizioCiclo = millis();

    abilitaMotori();
    attivaRele();

    Serial.println("=== INIZIO CICLO ===");

    // MOTORE 1: Va verso MAX
    stepperX.setMaxSpeed(velocitaMotore1);
    stepperX.moveTo(1000000); // Grande numero per raggiungere il finecorsa

    // MOTORE 2: Inizia oscillazione
    avviaMotore2();

    return;
  }

  // CONTROLLO SEMPLICE: se motore 1 è sul MIN, ciclo finito
  if (digitalRead(LIMIT_X_MIN) == LOW && motore1Completato)
  {
    // CICLO COMPLETATO
    cicloAttivo = false;
    motore2Completato = true;
    currentState = STATE_READY;

    stepperX.stop();
    stepperY.stop();
    disabilitaMotori();
    disattivaRele();

    // RESETTA POSIZIONE
    stepperX.setCurrentPosition(0);
    stepperY.setCurrentPosition(0);

    Serial.println("=== CICLO COMPLETATO ===");
    return;
  }

  // CONTROLLO MOTORE 1: se raggiunge MAX, torna al MIN
  if (digitalRead(LIMIT_X_MAX) == LOW && !motore1Completato)
  {
    Serial.println("X ha raggiunto MAX - torna al MIN");
    stepperX.setMaxSpeed(velocitaMotore1 * 2);
    stepperX.moveTo(-1000000); // Grande numero negativo per raggiungere il finecorsa MIN
    motore1Completato = true;
  }

  // CONTROLLO MOTORE 2: oscillazione continua
  if (!motore2Completato)
  {
    controllaMotore2();
  }
}

// =========== GESTIONE MOTORE 2 ===========
void avviaMotore2()
{
  stepperY.setMaxSpeed(velocitaMotore2);
  movimentoYAttivo = true;
  // Movimento iniziale verso MAX
  stepperY.moveTo(50000);
  Serial.println("Motore 2 iniziato avanti-indietro");
}

void controllaMotore2()
{
  if (!movimentoYAttivo) return;

  // Oscillazione semplice tra MIN e MAX
  if (digitalRead(LIMIT_Y_MAX) == LOW && stepperY.distanceToGo() > 0)
  {
    // Se sono su MAX, vado verso MIN
    stepperY.moveTo(-50000);
  }
  else if (digitalRead(LIMIT_Y_MIN) == LOW && stepperY.distanceToGo() < 0)
  {
    // Se sono su MIN, vado verso MAX
    stepperY.moveTo(50000);
  }
}

// =========== PAUSA E RIPRESA ===========
void pauseCycle()
{
    // 1. Ferma subito il Motore Y
    stepperY.stop();
    movimentoYAttivo = false;
    
    // 2. Imposta la retrocessione del Motore X di 3 giri (600 passi)
    stepperX.setMaxSpeed(1000); // Velocità moderata per la retrocessione
    stepperX.move(stepperX.currentPosition() - 600); // 3 giri indietro (200 * 3)

    disattivaRele();
    currentState = STATE_PAUSED; 
    
    Serial.println("Ciclo in pausa: X retrocede di 600 passi.");
}

void resumeCycle()
{
    currentState = STATE_RUNNING;
    movimentoYAttivo = true;
    // Ripristina il movimento oscillatorio del motore Y
    stepperY.moveTo(50000);
    attivaRele();
    Serial.println("Ciclo ripreso");
}

// =========== AGGIORNA DISPLAY ===========
void updateDisplay()
{
  display.clearDisplay();

  // Solo informazioni ESSENZIALI
  display.setTextSize(2);
  display.setCursor(0, 0);

  // STATO PRINCIPALE
  switch (currentState)
  {
  case STATE_OFF:
    display.print("OFF");
    break;
  case STATE_HOMING:
    display.print("HOMING");
    break;
  case STATE_READY:
    display.print("PRONTO");
    break;
  case STATE_RUNNING:
    display.print("IN CORSO");
    break;
  case STATE_PAUSED:
    display.print("PAUSA");
    break;
  case STATE_EMERGENCY:
    display.print("STOP");
    break;
  }

  // INFORMAZIONI IMPORTANTI
  display.setTextSize(1);

  // Righe 2-3: STATO MOTORI
  display.setCursor(0, 20);
  if (currentState == STATE_RUNNING)
  {
    display.print("M1:");
    display.print(digitalRead(LIMIT_X_MAX) == LOW ? "MAX" : digitalRead(LIMIT_X_MIN) == LOW ? "MIN"
                                                                                            : "IN MOV");
    display.setCursor(0, 30);
    display.print("M2:OSCILLA");
  }
  else
  {
    display.print("Premi START");
  }

  // Righe 4-5: POSIZIONI
  display.setCursor(0, 45);
  display.print("X:");
  display.print(stepperX.currentPosition());
  display.setCursor(70, 45);
  display.print("Y:");
  display.print(stepperY.currentPosition());

  // Righe 6-7: VELOCITA'
  display.setCursor(0, 55);
  display.print("V1:");
  display.print(velocitaMotore1);
  display.setCursor(70, 55);
  display.print("V2:");
  display.print(velocitaMotore2);

  // Righe 8-9: STATI
  display.setCursor(0, 70);
  display.print("RELE:");
  display.print(releAttivo ? "ON " : "OFF");
  display.setCursor(70, 70);
  display.print("MOT:");
  display.print(motoriAbilitati ? "ON" : "OFF");

  // Righe 10: FINE CORSA ATTIVI
  display.setCursor(0, 85);
  display.print("FSX:");
  display.print(digitalRead(LIMIT_X_MIN) == LOW ? "MIN" : digitalRead(LIMIT_X_MAX) == LOW ? "MAX"
                                                                                          : "NO");
  display.setCursor(70, 85);
  display.print("FSY:");
  display.print(digitalRead(LIMIT_Y_MIN) == LOW ? "MIN" : digitalRead(LIMIT_Y_MAX) == LOW ? "MAX"
                                                                                          : "NO");

  display.display();
}

// =========== LOOP PRINCIPALE ===========
void loop()
{
  checkButtons();
  aggiornaPotenziometri();

  // DEBUG OGNI 2 SECONDI
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 2000)
  {
    Serial.println("=== DEBUG ===");
    Serial.printf("Stato: %d, CicloAttivo: %d\n", currentState, cicloAttivo);
    Serial.printf("M1Completato: %d, M2Completato: %d\n", motore1Completato, motore2Completato);
    Serial.printf("Finecorsa X_MIN: %d, X_MAX: %d\n", digitalRead(LIMIT_X_MIN), digitalRead(LIMIT_X_MAX));
    Serial.printf("Finecorsa Y_MIN: %d, Y_MAX: %d\n", digitalRead(LIMIT_Y_MIN), digitalRead(LIMIT_Y_MAX));
    Serial.printf("Posizione X: %d, Y: %d\n", stepperX.currentPosition(), stepperY.currentPosition());
    Serial.printf("DistanceToGo X: %d, Y: %d\n", stepperX.distanceToGo(), stepperY.distanceToGo());
    Serial.println("=============");
    lastDebug = millis();
  }

  switch (currentState)
  {
  case STATE_HOMING:
    gestisciHoming();
    break;

  case STATE_RUNNING:
    cicloCoordinato();
    stepperX.run();
    stepperY.run();
    break;

  case STATE_PAUSED:
    // Durante la pausa, permette al motore X di completare la retrocessione
    stepperX.run();
    stepperY.run(); // Y dovrebbe essere fermo ma chiamiamo run() per sicurezza
    break;

  default:
    stepperX.run();
    stepperY.run();
    break;
  }

  updateDisplay();
  delay(10); // Ridotto il delay per migliore responsività
}