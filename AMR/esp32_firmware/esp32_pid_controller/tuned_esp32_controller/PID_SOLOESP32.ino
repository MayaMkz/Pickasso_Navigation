// ==========================================
// LABORATORIO PID AUTÓNOMO ESP32 (MOTORES A y B)
// Versión Corregida: PID Posicional Puro + Anti-Windup
// ==========================================

// --- Pines Motor A ---
const int R_EN_A = 16; const int L_EN_A = 4;
const int RPWM_A = 5;  const int LPWM_A = 17;
const int ENC_A_A = 23; const int ENC_B_A = 19; 

// --- Pines Motor B ---
const int R_EN_B = 27; const int L_EN_B = 14;
const int RPWM_B = 26; const int LPWM_B = 25;
const int ENC_A_B = 33; const int ENC_B_B = 32; 

volatile long encoderCountA = 0; volatile long encoderCountB = 0;
long lastCountA = 0; long lastCountB = 0;
unsigned long lastPIDTime = 0;

// --- Constantes Físicas ---
const float FACTOR_RPM = 0.06; 

// --- Variables de Control PID ---
float Kp = 0.0; 
float Ki = 0.0; 
float Kd = 0.0; 

float targetRPM = 0.0; 

float errorIntegralA = 0; float lastErrorA = 0; float currentPWMA = 0;
float errorIntegralB = 0; float lastErrorB = 0; float currentPWMB = 0;

void IRAM_ATTR updateEncoderA() { if (digitalRead(ENC_B_A) == LOW) encoderCountA++; else encoderCountA--; }
void IRAM_ATTR updateEncoderB() { if (digitalRead(ENC_B_B) == LOW) encoderCountB++; else encoderCountB--; }

void setup() {
  Serial.begin(115200);

  pinMode(R_EN_A, OUTPUT); pinMode(L_EN_A, OUTPUT); pinMode(RPWM_A, OUTPUT); pinMode(LPWM_A, OUTPUT);
  pinMode(R_EN_B, OUTPUT); pinMode(L_EN_B, OUTPUT); pinMode(RPWM_B, OUTPUT); pinMode(LPWM_B, OUTPUT);

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
  unsigned long currentMillis = millis();

  // 1. ESCUCHAR COMANDOS (Indestructible)
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    comando.toUpperCase(); 

    if (comando.length() > 0) {
      char letra = comando.charAt(0);
      
      String numeroStr = comando.substring(1);
      numeroStr.replace(",", ""); 
      numeroStr.trim();
      float valor = numeroStr.toFloat();

      if (letra == 'P') { Kp = valor; }
      else if (letra == 'I') { Ki = valor; }
      else if (letra == 'D') { Kd = valor; }
      else if (letra == 'M') { targetRPM = valor; }
      else if (letra == '0') { 
        targetRPM = 0; 
      }
    }
  }

  // 2. CICLO PID POSICIONAL (Cada 50ms)
  if (currentMillis - lastPIDTime >= 50) {
    noInterrupts(); 
    long cCountA = encoderCountA; 
    long cCountB = encoderCountB; 
    interrupts();

    float actualRpmA = abs(cCountA - lastCountA) * FACTOR_RPM;
    float actualRpmB = abs(cCountB - lastCountB) * FACTOR_RPM;
    lastCountA = cCountA; lastCountB = cCountB;

    if (targetRPM > 0) {
      // ==========================================
      // PID MOTOR A
      // ==========================================
      float errorA = targetRPM - actualRpmA;
      errorIntegralA += errorA;

      // Anti-Windup Seguro: Limita la memoria al tope físico de 255 de PWM
      if (Ki > 0) {
        if ((Ki * errorIntegralA) > 255) errorIntegralA = 255 / Ki;
        else if ((Ki * errorIntegralA) < 0) errorIntegralA = 0; // Evita memoria negativa en pruebas hacia adelante
      } else {
        errorIntegralA = 0; // Si no hay Ki, se limpia la memoria
      }

      float derivA = errorA - lastErrorA;
      lastErrorA = errorA;

      // LA CORRECCIÓN MÁGICA: Uso de "=" en lugar de "+="
      currentPWMA = (Kp * errorA) + (Ki * errorIntegralA) + (Kd * derivA);
      
      if (currentPWMA > 255) currentPWMA = 255; 
      if (currentPWMA < 0) currentPWMA = 0;

      analogWrite(RPWM_A, currentPWMA); analogWrite(LPWM_A, 0);

      // ==========================================
      // PID MOTOR B
      // ==========================================
      float errorB = targetRPM - actualRpmB;
      errorIntegralB += errorB;

      if (Ki > 0) {
        if ((Ki * errorIntegralB) > 255) errorIntegralB = 255 / Ki;
        else if ((Ki * errorIntegralB) < 0) errorIntegralB = 0;
      } else {
        errorIntegralB = 0;
      }

      float derivB = errorB - lastErrorB;
      lastErrorB = errorB;

      currentPWMB = (Kp * errorB) + (Ki * errorIntegralB) + (Kd * derivB);
      
      if (currentPWMB > 255) currentPWMB = 255; 
      if (currentPWMB < 0) currentPWMB = 0;

      analogWrite(RPWM_B, currentPWMB); analogWrite(LPWM_B, 0);

    } else {
      // Paro Suave
      currentPWMA = 0; currentPWMB = 0;
      errorIntegralA = 0; errorIntegralB = 0;
      lastErrorA = 0; lastErrorB = 0;
      analogWrite(RPWM_A, 0); analogWrite(LPWM_A, 0);
      analogWrite(RPWM_B, 0); analogWrite(LPWM_B, 0);
    }

    // 3. ENVIAR DATOS AL TRAZADOR SERIE
    Serial.print("Meta:"); Serial.print(targetRPM); Serial.print(",");
    Serial.print("MotorA:"); Serial.print(actualRpmA); Serial.print(",");
    Serial.print("MotorB:"); Serial.println(actualRpmB);
    
    lastPIDTime = currentMillis;
  }
}