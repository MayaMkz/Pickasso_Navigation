// --- Pines del Motor ---
const int R_EN = 27;
const int L_EN = 14;
const int RPWM = 26;
const int LPWM = 25;

// --- Pines del Encoder ---
const int ENC_A = 33; // Cable Negro (Fase A)
const int ENC_B = 32; // Cable Blanco (Fase B)

// Contador del encoder (debe ser 'volatile' porque se modifica en una interrupción)
volatile long encoderCount = 0;

// Variables para manejar el tiempo del motor sin usar delay()
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
int motorState = 0; // 0: Girar Derecha, 1: Pausa, 2: Girar Izquierda, 3: Pausa

// --- Rutina de Interrupción (ISR) del Encoder ---
// Se ejecuta automáticamente cada vez que la Fase A pasa de LOW a HIGH
void IRAM_ATTR updateEncoder() {
  // Según tu diagrama, si A sube y B está en LOW, es sentido horario (CW)
  if (digitalRead(ENC_B) == LOW) {
    encoderCount++; 
  } else {
    // Si A sube y B está en HIGH, es sentido antihorario (CCW)
    encoderCount--; 
  }
}

void setup() {
  Serial.begin(115200);

  // 1. Configuración del Motor
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  digitalWrite(RPWM, LOW);
  digitalWrite(LPWM, LOW);

  // 2. Configuración del Encoder (¡Vital usar INPUT_PULLUP por ser NPN!)
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // 3. Adjuntar la interrupción al pin A (se dispara cuando el voltaje sube: RISING)
  attachInterrupt(digitalPinToInterrupt(ENC_A), updateEncoder, RISING);
  
  Serial.println("Iniciando secuencia de motor y lectura de encoder...");
}

void loop() {
  currentMillis = millis();

  // --- Secuencia de Motor (No bloqueante) ---
  switch (motorState) {
    case 0: // Girar en una dirección
      digitalWrite(LPWM, LOW);
      digitalWrite(RPWM, HIGH);
      if (currentMillis - previousMillis >= 10000) { // 10 segundos
        motorState = 1;
        previousMillis = currentMillis;
      }
      break;
      
    case 1: // Pausa para perder inercia
      digitalWrite(RPWM, LOW);
      digitalWrite(LPWM, LOW);
      if (currentMillis - previousMillis >= 1000) {  // 1 segundo
        motorState = 2;
        previousMillis = currentMillis;
      }
      break;
      
    case 2: // Girar en dirección opuesta
      digitalWrite(RPWM, LOW);
      digitalWrite(LPWM, HIGH);
      if (currentMillis - previousMillis >= 10000) { // 10 segundos
        motorState = 3;
        previousMillis = currentMillis;
      }
      break;
      
    case 3: // Pausa antes de repetir
      digitalWrite(RPWM, LOW);
      digitalWrite(LPWM, LOW);
      if (currentMillis - previousMillis >= 1000) {  // 1 segundo
        motorState = 0; // Reiniciar ciclo
        previousMillis = currentMillis;
      }
      break;
  }

  // --- Imprimir lectura del Encoder ---
  // Imprimimos la posición cada 100 milisegundos para no saturar el Monitor Serie
  static unsigned long lastPrint = 0;
  if (currentMillis - lastPrint >= 100) {
    Serial.print("Posicion del Encoder: ");
    Serial.println(encoderCount);
    lastPrint = currentMillis;
  }
}