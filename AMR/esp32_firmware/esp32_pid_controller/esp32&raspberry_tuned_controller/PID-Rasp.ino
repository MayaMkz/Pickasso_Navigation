// ==========================================
// CÓDIGO ESP32 FINAL: CLIENTE PID Y ODOMETRÍA
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
const float FACTOR_RPM = 0.06; // Para 20000 pulsos cada 50ms

// ==========================================
// 2. TUS GANANCIAS PID (¡Pon tus valores aquí!)
// ==========================================
float Kp = 4.0;  // Cambia esto por tu valor
float Ki = 1.0;  // Cambia esto por tu valor
float Kd = 0.0; // Cambia esto por tu valor

float targetRpmA = 0.0; float targetRpmB = 0.0;
int dirA = 0; int dirB = 0;

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

  // 1. ESCUCHAR A LA RASPBERRY PI
  // Formato: A,1,50 (Motor A, Adelante, 50 RPM)
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    
    int coma1 = comando.indexOf(',');
    int coma2 = comando.indexOf(',', coma1 + 1);

    if (coma1 != -1 && coma2 != -1) {
      char motor = comando.charAt(0);
      int direccion = comando.substring(coma1 + 1, coma2).toInt();
      float rpm = comando.substring(coma2 + 1).toFloat();

      if (motor == 'A') {
        dirA = direccion; targetRpmA = (direccion == 0) ? 0 : rpm;
        if(direccion == 0) { currentPWMA = 0; errorIntegralA = 0; analogWrite(RPWM_A, 0); analogWrite(LPWM_A, 0); }
      } 
      else if (motor == 'B') {
        dirB = direccion; targetRpmB = (direccion == 0) ? 0 : rpm;
        if(direccion == 0) { currentPWMB = 0; errorIntegralB = 0; analogWrite(RPWM_B, 0); analogWrite(LPWM_B, 0); }
      }
    }
  }

  // 2. LAZO DE CONTROL PID (50ms)
  if (currentMillis - lastPIDTime >= 50) {
    noInterrupts(); 
    long cCountA = encoderCountA; 
    long cCountB = encoderCountB; 
    interrupts();

    float actualRpmA = abs(cCountA - lastCountA) * FACTOR_RPM;
    float actualRpmB = abs(cCountB - lastCountB) * FACTOR_RPM;
    lastCountA = cCountA; lastCountB = cCountB;

    // --- PID MOTOR A ---
    if (targetRpmA > 0) {
      float errorA = targetRpmA - actualRpmA;
      errorIntegralA += errorA;
      
      if (Ki > 0) {
        if ((Ki * errorIntegralA) > 255) errorIntegralA = 255 / Ki;
        else if ((Ki * errorIntegralA) < 0) errorIntegralA = 0;
      } else { errorIntegralA = 0; }

      float derivA = errorA - lastErrorA; lastErrorA = errorA;
      currentPWMA = (Kp * errorA) + (Ki * errorIntegralA) + (Kd * derivA);
      
      if (currentPWMA > 255) currentPWMA = 255; if (currentPWMA < 0) currentPWMA = 0;
      if (dirA == 1) { analogWrite(RPWM_A, currentPWMA); analogWrite(LPWM_A, 0); }
      else if (dirA == 2) { analogWrite(RPWM_A, 0); analogWrite(LPWM_A, currentPWMA); }
    }

    // --- PID MOTOR B ---
    if (targetRpmB > 0) {
      float errorB = targetRpmB - actualRpmB;
      errorIntegralB += errorB;
      
      if (Ki > 0) {
        if ((Ki * errorIntegralB) > 255) errorIntegralB = 255 / Ki;
        else if ((Ki * errorIntegralB) < 0) errorIntegralB = 0;
      } else { errorIntegralB = 0; }

      float derivB = errorB - lastErrorB; lastErrorB = errorB;
      currentPWMB = (Kp * errorB) + (Ki * errorIntegralB) + (Kd * derivB);
      
      if (currentPWMB > 255) currentPWMB = 255; if (currentPWMB < 0) currentPWMB = 0;
      if (dirB == 1) { analogWrite(RPWM_B, currentPWMB); analogWrite(LPWM_B, 0); }
      else if (dirB == 2) { analogWrite(RPWM_B, 0); analogWrite(LPWM_B, currentPWMB); }
    }

    // 3. TELEMETRÍA (Mandamos los pulsos acumulados para la odometría de la Raspberry)
    Serial.print("A:"); Serial.print(cCountA);
    Serial.print(",B:"); Serial.println(cCountB);
    
    lastPIDTime = currentMillis;
  }
}