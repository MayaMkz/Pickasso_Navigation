// ==========================================
// CÓDIGO ESP32: OYENTE CON PWM FIJO A 130
// ==========================================

// --- Pines del Motor A ---
const int R_EN_A = 16;
const int L_EN_A = 4;
const int RPWM_A = 5;
const int LPWM_A = 17;

// --- Pines del Encoder A ---
const int ENC_A_A = 23; 
const int ENC_B_A = 19; 

// --- Pines del Motor B ---
const int R_EN_B = 27;
const int L_EN_B = 14;
const int RPWM_B = 26;
const int LPWM_B = 25;

// --- Pines del Encoder B ---
const int ENC_A_B = 33; 
const int ENC_B_B = 32; 

volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

unsigned long currentMillis = 0;
unsigned long lastPrint = 0;

// Variables de Control
const int VELOCIDAD_FIJA = 150; // <-- PWM Fijo

unsigned long startTimeA = 0; 
unsigned long durationA = 0; 
bool isMovingA = false;

unsigned long startTimeB = 0; 
unsigned long durationB = 0; 
bool isMovingB = false;

// --- Interrupciones ---
void IRAM_ATTR updateEncoderA() {
  if (digitalRead(ENC_B_A) == LOW) encoderCountA++; else encoderCountA--; 
}
void IRAM_ATTR updateEncoderB() {
  if (digitalRead(ENC_B_B) == LOW) encoderCountB++; else encoderCountB--; 
}

void setup() {
  Serial.begin(115200);

  pinMode(R_EN_A, OUTPUT); pinMode(L_EN_A, OUTPUT);
  pinMode(RPWM_A, OUTPUT); pinMode(LPWM_A, OUTPUT);
  pinMode(R_EN_B, OUTPUT); pinMode(L_EN_B, OUTPUT);
  pinMode(RPWM_B, OUTPUT); pinMode(LPWM_B, OUTPUT);

  digitalWrite(R_EN_A, HIGH); digitalWrite(L_EN_A, HIGH);
  digitalWrite(R_EN_B, HIGH); digitalWrite(L_EN_B, HIGH);

  analogWrite(RPWM_A, 0); analogWrite(LPWM_A, 0);
  analogWrite(RPWM_B, 0); analogWrite(LPWM_B, 0);

  pinMode(ENC_A_A, INPUT_PULLUP); pinMode(ENC_B_A, INPUT_PULLUP);
  pinMode(ENC_A_B, INPUT_PULLUP); pinMode(ENC_B_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_A), updateEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_B), updateEncoderB, RISING);
}

void loop() {
  currentMillis = millis();

  // 1. ESCUCHAR A LA RASPBERRY
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    int coma1 = comando.indexOf(',');
    int coma2 = comando.indexOf(',', coma1 + 1);

    if (coma1 != -1 && coma2 != -1) {
      char motor = comando.charAt(0);
      int direccion = comando.substring(coma1 + 1, coma2).toInt();
      long tiempo = comando.substring(coma2 + 1).toInt();

      if (motor == 'A') {
        if (direccion == 0) {
          isMovingA = false; 
          analogWrite(RPWM_A, 0); analogWrite(LPWM_A, 0);
        } else {
          startTimeA = currentMillis; durationA = tiempo; isMovingA = true;
          if (direccion == 1) { analogWrite(RPWM_A, VELOCIDAD_FIJA); analogWrite(LPWM_A, 0); }
          else if (direccion == 2) { analogWrite(RPWM_A, 0); analogWrite(LPWM_A, VELOCIDAD_FIJA); }
        }
      } 
      else if (motor == 'B') {
        if (direccion == 0) {
          isMovingB = false; 
          analogWrite(RPWM_B, 0); analogWrite(LPWM_B, 0);
        } else {
          startTimeB = currentMillis; durationB = tiempo; isMovingB = true;
          if (direccion == 1) { analogWrite(RPWM_B, VELOCIDAD_FIJA); analogWrite(LPWM_B, 0); }
          else if (direccion == 2) { analogWrite(RPWM_B, 0); analogWrite(LPWM_B, VELOCIDAD_FIJA); }
        }
      }
    }
  }

  // 2. PARO AUTOMÁTICO POR TIEMPO
  if (isMovingA && (currentMillis - startTimeA >= durationA)) {
    isMovingA = false; analogWrite(RPWM_A, 0); analogWrite(LPWM_A, 0);
  }
  if (isMovingB && (currentMillis - startTimeB >= durationB)) {
    isMovingB = false; analogWrite(RPWM_B, 0); analogWrite(LPWM_B, 0);
  }

  // 3. REPORTAR ENCODERS
  if (currentMillis - lastPrint >= 100) {
    noInterrupts();
    long countA = encoderCountA;
    long countB = encoderCountB;
    interrupts();

    Serial.print("A:"); Serial.print(countA);
    Serial.print(",B:"); Serial.println(countB);
    
    lastPrint = currentMillis;
  }
}