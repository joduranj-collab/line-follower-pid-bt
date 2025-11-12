#include <QTRSensors.h>

// === Definición de pines de motores ===
#define IN1 4
#define IN2 9
#define ENA 5  // PWM Motor 1 (Izquierdo)
#define IN3 7
#define IN4 8
#define ENB 6  // PWM Motor 2 (Derecho)
#define IR_PIN 11 // Pin para el IR (siempre en HIGH)

QTRSensors qtr;

// === PID ===
float Kp = 0.5, Ki = 0.000, Kd = 1.0;
int P, I, D, lastError = 0;

// === Centro de línea ===
int centerLine = 3900;

// === Velocidades ===
uint8_t maxspeedA = 55, maxspeedB = 55;
uint8_t basespeedA = 50, basespeedB = 50;
float factorA = 1.0, factorB = 1.0;

// === Sensores ===
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// === Buffer de Bluetooth ===
String btBuffer = "";

// === Prototipos ===
void PID_control();
void motores(int izq, int der);
void handleBluetooth();
void processCommand(String cmd);

void setup() {
  Serial.begin(9600);
  delay(200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IR_PIN, OUTPUT); digitalWrite(IR_PIN, HIGH);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7,A6,A5,A4,A3,A2,A1,A0}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  for (uint16_t i=0;i<100;i++) qtr.calibrate();

  Serial.println("=== HC-05 listo ===");
  Serial.println("Comandos:");
  Serial.println("  KP= KI= KD=");
  Serial.println("  CENTER= o C=");
  Serial.println("  BASEA= BASEB= BASE= o B=");
  Serial.println("  MAXA= MAXB=");
  Serial.println("  READ");
}

void loop() {
  handleBluetooth();
  PID_control();
  delay(50);
}

// =================== BT ===================
void handleBluetooth() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Si llega fin de línea (CR o LF)
    if (c == '\r' || c == '\n') {
      btBuffer.trim();
      if (btBuffer.length() > 0) {
        processCommand(btBuffer);
      }
      btBuffer = "";   // limpiar para el siguiente comando
    } else {
      btBuffer += c;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // Versión en mayúsculas para comparar comandos
  String up = cmd;
  up.toUpperCase();

  int eqPos = up.indexOf('=');
  String key = (eqPos >= 0) ? up.substring(0, eqPos) : up;
  String valueStr = (eqPos >= 0) ? cmd.substring(eqPos + 1) : "";

  // --- PID ---
  if (key == "KP") {
    Kp = valueStr.toFloat();
    Serial.print("Kp -> "); Serial.println(Kp);
  }
  else if (key == "KI") {
    Ki = valueStr.toFloat();
    Serial.print("Ki -> "); Serial.println(Ki);
  }
  else if (key == "KD") {
    Kd = valueStr.toFloat();
    Serial.print("Kd -> "); Serial.println(Kd);
  }

  // --- Centro ---
  else if (key == "CENTER" || key == "C") {
    centerLine = valueStr.toInt();
    Serial.print("Centro -> "); Serial.println(centerLine);
  }

  // --- Velocidades base individuales ---
  else if (key == "BASEA") {
    int nuevo = constrain(valueStr.toInt(), 0, 255);
    basespeedA = (uint8_t)nuevo;
    Serial.print("Base A -> "); Serial.println(basespeedA);
  }
  else if (key == "BASEB") {
    int nuevo = constrain(valueStr.toInt(), 0, 255);
    basespeedB = (uint8_t)nuevo;
    Serial.print("Base B -> "); Serial.println(basespeedB);
  }

  // --- NUEVO: ambas bases a la vez ---
  else if (key == "BASE" || key == "B") {
    int nuevo = constrain(valueStr.toInt(), 0, 255);
    basespeedA = (uint8_t)nuevo;
    basespeedB = (uint8_t)nuevo;
    Serial.print("Bases A y B -> "); Serial.println(nuevo);
  }

  // --- Límites máximos ---
  else if (key == "MAXA") {
    int nuevo = constrain(valueStr.toInt(), 0, 255);
    maxspeedA = (uint8_t)nuevo;
    Serial.print("Max A -> "); Serial.println(maxspeedA);
  }
  else if (key == "MAXB") {
    int nuevo = constrain(valueStr.toInt(), 0, 255);
    maxspeedB = (uint8_t)nuevo;
    Serial.print("Max B -> "); Serial.println(maxspeedB);
  }

  // --- Lectura general ---
  else if (up == "READ") {
    uint16_t position = qtr.readLineBlack(sensorValues);
    int error = position - centerLine;

    Serial.print("Posicion: "); Serial.print(position);
    Serial.print("  Centro: "); Serial.print(centerLine);
    Serial.print("  Error: "); Serial.println(error);

    Serial.print("PID: Kp="); Serial.print(Kp);
    Serial.print(" Ki="); Serial.print(Ki);
    Serial.print(" Kd="); Serial.println(Kd);

    Serial.print("Speeds: baseA="); Serial.print(basespeedA);
    Serial.print(" baseB="); Serial.print(basespeedB);
    Serial.print(" maxA="); Serial.print(maxspeedA);
    Serial.print(" maxB="); Serial.println(maxspeedB);
  }

  else {
    Serial.print("Comando no valido: ");
    Serial.println(cmd);
  }
}

// =================== CONTROL ===================
void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - centerLine;

  P = error;
  I += error;
  D = error - lastError;
  lastError = error;

  int motorspeed = (P*Kp) + (I*Ki) + (D*Kd);

  int motorspeedA = (basespeedA + motorspeed) * factorA;
  int motorspeedB = (basespeedB - motorspeed) * factorB;

  motorspeedA = constrain(motorspeedA,0,maxspeedA);
  motorspeedB = constrain(motorspeedB,0,maxspeedB);

  motores(motorspeedA, motorspeedB);
}

void motores(int izq, int der) {
  digitalWrite(IN1, izq>=0 ? HIGH : LOW);
  digitalWrite(IN2, izq>=0 ? LOW  : HIGH);
  analogWrite(ENA, abs(izq));
  
  digitalWrite(IN3, der>=0 ? HIGH : LOW);
  digitalWrite(IN4, der>=0 ? LOW  : HIGH);
  analogWrite(ENB, abs(der));
}

