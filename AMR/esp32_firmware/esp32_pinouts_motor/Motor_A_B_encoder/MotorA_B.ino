// ==========================================
// --- Pines del Motor A ---
const int R_EN_A = 16;
const int L_EN_A = 4;
const int RPWM_A = 5;
const int LPWM_A = 17;

// --- Pines del Encoder A ---
const int ENC_A_A = 23; // Señal A
const int ENC_B_A = 19; // Señal B
// ==========================================

// ==========================================
// --- Pines del Motor B ---
const int R_EN_B = 27;
const int L_EN_B = 14;
const int RPWM_B = 26;
const int LPWM_B = 25;

// --- Pines del Encoder B ---
const int ENC_A_B = 33; // Señal A
const int ENC_B_B = 32; // Señal B
// ==========================================

// Contadores independientes para cada encoder
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

// Variables de tiempo y estado
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
int motorState = 0; // 0: Girar Derecha, 1: Pausa, 2: Girar Izquierda, 3: Pausa

// --- Rutina de Interrupción Motor A ---
void IRAM_ATTR updateEncoderA() {
  if (digitalRead(ENC_B_A) == LOW) {
    encoderCountA++; 
  } else {
    encoderCountA--; 
  }
}

// --- Rutina de Interrupción Motor B ---
void IRAM_ATTR updateEncoderB() {
  if (digitalRead(ENC_B_B) == LOW) {
    encoderCountB++; 
  } else {
    encoderCountB--; 
  }
}

void setup() {
  Serial.begin(115200);

  // 1. Configurar pines de salida para AMBOS motores
  pinMode(R_EN_A, OUTPUT); pinMode(L_EN_A, OUTPUT);
  pinMode(RPWM_A, OUTPUT); pinMode(LPWM_A, OUTPUT);
  
  pinMode(R_EN_B, OUTPUT); pinMode(L_EN_B, OUTPUT);
  pinMode(RPWM_B, OUTPUT); pinMode(LPWM_B, OUTPUT);

  // Habilitar ambos puentes H
  digitalWrite(R_EN_A, HIGH); digitalWrite(L_EN_A, HIGH);
  digitalWrite(R_EN_B, HIGH); digitalWrite(L_EN_B, HIGH);

  // Asegurar que ambos inicien detenidos
  digitalWrite(RPWM_A, LOW); digitalWrite(LPWM_A, LOW);
  digitalWrite(RPWM_B, LOW); digitalWrite(LPWM_B, LOW);

  // 2. Configurar pines de entrada para AMBOS encoders con PULLUP
  pinMode(ENC_A_A, INPUT_PULLUP); pinMode(ENC_B_A, INPUT_PULLUP);
  pinMode(ENC_A_B, INPUT_PULLUP); pinMode(ENC_B_B, INPUT_PULLUP);

  // 3. Adjuntar interrupciones independientes
  attachInterrupt(digitalPinToInterrupt(ENC_A_A), updateEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_B), updateEncoderB, RISING);
  
  Serial.println("Iniciando prueba dual de motores y encoders...");
}

void loop() {
  currentMillis = millis();

  // --- Secuencia de Motores Sincronizada ---
  switch (motorState) {
    case 0: // Girar ambos en una dirección
      digitalWrite(LPWM_A, LOW); digitalWrite(RPWM_A, HIGH);
      digitalWrite(LPWM_B, LOW); digitalWrite(RPWM_B, HIGH);
      if (currentMillis - previousMillis >= 10000) { 
        motorState = 1;
        previousMillis = currentMillis;
      }
      break;
      
    case 1: // Pausa para perder inercia
      digitalWrite(RPWM_A, LOW); digitalWrite(LPWM_A, LOW);
      digitalWrite(RPWM_B, LOW); digitalWrite(LPWM_B, LOW);
      if (currentMillis - previousMillis >= 1000) {  
        motorState = 2;
        previousMillis = currentMillis;
      }
      break;
      
    case 2: // Girar ambos en dirección opuesta
      digitalWrite(RPWM_A, LOW); digitalWrite(LPWM_A, HIGH);
      digitalWrite(RPWM_B, LOW); digitalWrite(LPWM_B, HIGH);
      if (currentMillis - previousMillis >= 10000) { 
        motorState = 3;
        previousMillis = currentMillis;
      }
      break;
      
    case 3: // Pausa antes de repetir
      digitalWrite(RPWM_A, LOW); digitalWrite(LPWM_A, LOW);
      digitalWrite(RPWM_B, LOW); digitalWrite(LPWM_B, LOW);
      if (currentMillis - previousMillis >= 1000) {  
        motorState = 0; 
        previousMillis = currentMillis;
      }
      break;
  }

  // --- Imprimir lectura de ambos Encoders ---
  static unsigned long lastPrint = 0;
  if (currentMillis - lastPrint >= 100) {
    
    // Pausar interrupciones muy brevemente para copiar los valores de forma segura
    noInterrupts();
    long currentCountA = encoderCountA;
    long currentCountB = encoderCountB;
    interrupts();

    Serial.print("Encoder A: ");
    Serial.print(currentCountA);
    Serial.print("\t|\tEncoder B: ");
    Serial.println(currentCountB);
    
    lastPrint = currentMillis;
  }
}
