#include <Arduino.h>
#include <L298N.h>
#include <HardwareSerial.h>

// ========== CONFIGURACIÓN DEL RADAR HLK-LD2420 ==========
HardwareSerial RadarSerial(2);
const int RADAR_RX = 16;  // ESP32 RX (conecta a OT1/TX del radar)
const int RADAR_TX = 17;  // ESP32 TX (conecta a RX del radar)
const int RADAR_OT2 = 23; // Pin de detección digital (OT2)

// Variables del Radar
bool personDetected = false;
int distanceGate = 0;
int distanceCm = 0;
String radarBuffer = "";
unsigned long lastRadarUpdate = 0;

// Distancia de seguridad en cm (40cm según requerimiento)
const int DISTANCIA_SEGURIDAD = 40;

// Configuración de Gates del HLK-LD2420
// Cada gate representa aprox. 20-25cm dependiendo del modelo
const int CM_POR_GATE = 20;  // Ajustar según tu radar específico
const int MAX_GATE_SEGURIDAD = 2;  // Gate 0-2 = 0-60cm aprox

// ========== CONFIGURACIÓN DE MOTORES L298N ==========
const unsigned int IN3 = 22;
const unsigned int IN4 = 21;
const unsigned int ENB = 5;
const unsigned int ENA = 4;
const unsigned int IN1 = 16;
const unsigned int IN2 = 17;

L298N motorL(ENA, IN1, IN2);
L298N motorR(ENB, IN3, IN4);

// ========== SENSORES HALL ==========
const uint8_t HALL_L = 35;
const uint8_t HALL_R = 34;

volatile unsigned int cntL = 0;
volatile unsigned int cntR = 0;

// ========== PARÁMETROS FÍSICOS DEL ROBOT ==========
const int PULSOS_POR_VUELTA = 8;
const float DIAMETRO_RUEDA = 6.5;
const float CIRCUNFERENCIA = 3.1416 * DIAMETRO_RUEDA;
const float DISTANCIA_POR_PULSO = CIRCUNFERENCIA / PULSOS_POR_VUELTA;

// ========== PARÁMETROS DE LA PISTA ==========
const float TAMANO_CASILLA = 170.0;

// ========== CALIBRACIÓN PARA GIRO 90° ==========
const int PULSOS_GIRO_90 = 7;
const int VELOCIDAD_GIRO = 80;

// ========== CALIBRACIÓN PARA AVANCE DE 170cm ==========
const int PULSOS_AVANCE_170CM = (int)(TAMANO_CASILLA / DISTANCIA_POR_PULSO);

// Control de velocidad (reducido 15%)
const int BASE_SPEED = 102;
const int MIN_SPEED = 43;
const int MAX_SPEED = 217;

int pwmL = 100;
int pwmR = 125;

// ========== PARÁMETROS DE DESACELERACIÓN ==========
const int PULSOS_INICIO_DECEL = 6;
const int VELOCIDAD_MINIMA_DECEL = 60;

// Control PID
const unsigned long CONTROL_INTERVAL_MS = 200;
const int MAX_DELTA_PER_STEP = 20;
float Kp = 25.0f;
float Ki = 0.0f;
float Kd = 2.0f;

float integL = 0.0f;
float integR = 0.0f;
int lastErrL = 0;
int lastErrR = 0;
unsigned long lastControlT = 0;
float targetRevsPerSec = 1.5f;
const int PULSOS_POR_REV = 8;

// ========== MATRIZ 3x3 ==========
const int ROWS = 3;
const int COLS = 3;
int grid[3][3] = {
  { 0, 0, 0},  // Fila Y=0 (superior)
  { 1, 1, 0},  // Fila Y=1
  {-1, 0, 0}   // Fila Y=2 (inferior con flecha en X=0)
};

int posX = 0;
int posY = 2;
enum Direction {NORTH, EAST, SOUTH, WEST};
Direction dir = EAST;

int targetX = 0;
int targetY = 0;

// ========== INTERRUPCIONES HALL ==========
void IRAM_ATTR onHallL() { cntL++; }
void IRAM_ATTR onHallR() { cntR++; }

// ========== FUNCIONES DEL RADAR ==========

/**
 * Lee y procesa los datos del radar en formato de texto
 */
void readRadarData() {
  while (RadarSerial.available()) {
    char c = RadarSerial.read();
    
    if (c == '\n') {
      radarBuffer.trim();
      
      //if (radarBuffer.length() > 0) {
      //  parseRadarCommand(radarBuffer);
      //}
      
      radarBuffer = "";
    }
    else if (c != '\r') {
      radarBuffer += c;
    }
  }
}

/**
 * Parsea los comandos del radar HLK-LD2420
 */
void parseRadarCommand(String data) {
  data.trim();
  
  if (data == "ON") {
    personDetected = true;
    Serial.println("⚠  PERSONA DETECTADA");
  }
  else if (data == "OFF") {
    personDetected = false;
    distanceGate = 0;
    distanceCm = 0;
    Serial.println("✓ Camino libre");
  }
  else if (data.startsWith("Range ")) {
    String rangeStr = data.substring(6);
    distanceGate = rangeStr.toInt();
    distanceCm = distanceGate * CM_POR_GATE;  // Conversión corregida
    
    Serial.print("📏 Distancia: Gate ");
    Serial.print(distanceGate);
    Serial.print(" (~");
    Serial.print(distanceCm);
    Serial.println(" cm)");
  }
}

/**
 * Verifica si hay una persona en la zona de seguridad (40cm)
 * Retorna true si debe detenerse
 */
bool checkSafetyZone() {
  readRadarData();
  
  // MÉTODO 1: Lectura directa del pin OT2 (MÁS CONFIABLE)
  bool pinDetection = digitalRead(RADAR_OT2) == HIGH;
  
  // MÉTODO 2: Verificar por comando serial "ON"
  bool serialDetection = personDetected;
  
  // Si CUALQUIERA de los dos métodos detecta presencia, detener
  if (pinDetection || serialDetection) {
    if (pinDetection) {
      Serial.println("🚨 DETECCIÓN POR PIN OT2");
    }
    if (serialDetection) {
      Serial.println("🚨 DETECCIÓN POR COMANDO SERIAL");
    }
    return true; // Debe detenerse
  }
  
  return false; // Puede continuar
}

/**
 * Espera hasta que el camino esté libre
 */
void waitForClearPath() {
  bool wasBlocked = false;
  
  while (checkSafetyZone()) {
    if (!wasBlocked) {
      Serial.println("╔════════════════════════════════════════╗");
      Serial.println("║  ⛔ DETENIDO - PERSONA DETECTADA      ║");
      Serial.println("║     EN ZONA DE SEGURIDAD              ║");
      Serial.println("╚════════════════════════════════════════╝");
      Serial.println("🔴 ESTADO: BLOQUEADO");
      wasBlocked = true;
    }
    
    // Continuar leyendo datos del radar
    readRadarData();
    
    // Mostrar estado cada segundo
    unsigned long currentTime = millis();
    if (currentTime - lastRadarUpdate >= 1000) {
      lastRadarUpdate = currentTime;
      bool pinState = digitalRead(RADAR_OT2) == HIGH;
      Serial.print("🔴 BLOQUEADO | PIN OT2: ");
      Serial.print(pinState ? "HIGH" : "LOW");
      Serial.print(" | Serial: ");
      Serial.print(personDetected ? "ON" : "OFF");
      if (distanceGate > 0) {
        Serial.print(" | Gate: ");
        Serial.print(distanceGate);
      }
      Serial.println();
    }
    
    delay(100);
  }
  
  if (wasBlocked) {
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  ✓ CAMINO DESPEJADO                   ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println("🟢 ESTADO: LIBRE - Reanudando navegación");
    delay(500); // Pequeña pausa antes de continuar
  }
}

// ========== FUNCIONES BÁSICAS DEL ROBOT ==========

void findInitialPosition() {
  for (int y = 0; y < ROWS; y++) {
    for (int x = 0; x < COLS; x++) {
      if (grid[y][x] == -1) {
        posX = x;
        posY = y;
        grid[y][x] = 0;
        Serial.print("Posición inicial (flecha): (");
        Serial.print(posX);
        Serial.print(", ");
        Serial.print(posY);
        Serial.println(")");
        return;
      }
    }
  }
}

void stopMotors() {
  motorL.stop();
  motorR.stop();
  delay(200);
}

bool canMoveForward() {
  int nextX = posX;
  int nextY = posY;
  switch (dir) {
    case NORTH: nextY--; break;
    case EAST:  nextX++; break;
    case SOUTH: nextY++; break;
    case WEST:  nextX--; break;
  }
  return nextX >= 0 && nextX < COLS && nextY >= 0 && nextY < ROWS && grid[nextY][nextX] == 0;
}

int targetCountsPerInterval() {
  float secs = CONTROL_INTERVAL_MS / 1000.0f;
  return (int)round(targetRevsPerSec * secs * PULSOS_POR_REV);
}

// ========== MOVIMIENTO CON DETECCIÓN DE PERSONAS ==========

void moveForwardCalibratedDistance() {
  if (!canMoveForward()) return;
  
  stopMotors();
  delay(300);
  
  // Verificar zona de seguridad antes de iniciar
  waitForClearPath();
  
  noInterrupts();
  cntL = 0;
  cntR = 0;
  interrupts();
  
  Serial.println("------------------------------------");
  Serial.print("Avanzando 170cm. Objetivo: ");
  Serial.print(PULSOS_AVANCE_170CM);
  Serial.println(" pulsos");
  Serial.println("🟢 ESTADO: LIBRE - Avanzando");
  
  motorL.setSpeed(pwmL);
  motorR.setSpeed(pwmR);
  motorL.forward();
  motorR.forward();
  
  int velocidadActualL = pwmL;
  int velocidadActualR = pwmR;
  
  while (true) {
    // *** VERIFICACIÓN CONTINUA DE SEGURIDAD ***
    if (checkSafetyZone()) {
      Serial.println("⚠  PERSONA DETECTADA DURANTE MOVIMIENTO");
      Serial.println("🔴 ESTADO: BLOQUEADO");
      stopMotors();
      waitForClearPath();
      
      // Reanudar movimiento
      Serial.println("🟢 ESTADO: LIBRE - Reanudando avance");
      motorL.setSpeed(velocidadActualL);
      motorR.setSpeed(velocidadActualR);
      motorL.forward();
      motorR.forward();
    }
    
    noInterrupts();
    unsigned int pulsosL = cntL;
    unsigned int pulsosR = cntR;
    interrupts();
    
    int promedio = (pulsosL + pulsosR) / 2;
    
    // DESACELERACIÓN PROGRESIVA
    if (promedio >= (PULSOS_AVANCE_170CM - PULSOS_INICIO_DECEL)) {
      int pulsosRestantes = PULSOS_AVANCE_170CM - promedio;
      if (pulsosRestantes > 0) {
        float factor = (float)pulsosRestantes / PULSOS_INICIO_DECEL;
        velocidadActualL = VELOCIDAD_MINIMA_DECEL + (int)((pwmL - VELOCIDAD_MINIMA_DECEL) * factor);
        velocidadActualR = VELOCIDAD_MINIMA_DECEL + (int)((pwmR - VELOCIDAD_MINIMA_DECEL) * factor);
        
        motorL.setSpeed(velocidadActualL);
        motorR.setSpeed(velocidadActualR);
        
        Serial.print("DESACELERANDO - Vel: ");
        Serial.print(velocidadActualL);
        Serial.print("/");
        Serial.print(velocidadActualR);
        Serial.print(" | Restantes: ");
        Serial.println(pulsosRestantes);
      }
    }
    
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
      Serial.print("🟢 LIBRE | PL: ");
      Serial.print(pulsosL);
      Serial.print(" | PR: ");
      Serial.print(pulsosR);
      Serial.print(" | Promedio: ");
      Serial.print(promedio);
      Serial.print(" / ");
      Serial.println(PULSOS_AVANCE_170CM);
      lastPrint = millis();
    }
    
    if (promedio >= PULSOS_AVANCE_170CM) {
      break;
    }
    delay(10);
  }
  
  stopMotors();
  delay(300);
  
  switch (dir) {
    case NORTH: posY--; break;
    case EAST:  posX++; break;
    case SOUTH: posY++; break;
    case WEST:  posX--; break;
  }
  
  Serial.print("Nueva posición: (");
  Serial.print(posX);
  Serial.print(", ");
  Serial.print(posY);
  Serial.println(")");
  Serial.println("------------------------------------");
}

// ========== GIROS CON DETECCIÓN ==========

void girarDerechaCalibrado() {
  stopMotors();
  delay(300);
  
  // Verificar seguridad antes de girar
  waitForClearPath();
  
  noInterrupts();
  cntL = 0;
  cntR = 0;
  interrupts();
  
  Serial.println("------------------------------------");
  Serial.println(">>> RESET de pulsos antes del giro");
  Serial.print("Iniciando giro DERECHA 90°. Objetivo: ");
  Serial.print(PULSOS_GIRO_90);
  Serial.println(" pulsos");
  Serial.println("🟢 ESTADO: LIBRE - Girando derecha");
  
  motorL.setSpeed(55);
  motorR.setSpeed(77);
  motorL.forward();
  motorR.backward();
  
  int velActual = VELOCIDAD_GIRO;
  const int PULSOS_DECEL_GIRO = 2;
  
  while (true) {
    // Verificar seguridad durante giro
    if (checkSafetyZone()) {
      Serial.println("🔴 ESTADO: BLOQUEADO durante giro");
      stopMotors();
      waitForClearPath();
      Serial.println("🟢 ESTADO: LIBRE - Continuando giro");
      motorL.setSpeed(velActual);
      motorR.setSpeed(velActual);
      motorL.forward();
      motorR.backward();
    }
    
    noInterrupts();
    unsigned int pulsosL = cntL;
    unsigned int pulsosR = cntR;
    interrupts();
    
    int promedio = (pulsosL + pulsosR) / 2;
    
    if (promedio >= (PULSOS_GIRO_90 - PULSOS_DECEL_GIRO)) {
      int restantes = PULSOS_GIRO_90 - promedio;
      if (restantes > 0) {
        float factor = (float)restantes / PULSOS_DECEL_GIRO;
        velActual = 51 + (int)((VELOCIDAD_GIRO - 51) * factor);
        motorL.setSpeed(velActual);
        motorR.setSpeed(velActual);
      }
    }
    
    Serial.print("🟢 LIBRE | PL: ");
    Serial.print(pulsosL);
    Serial.print(" | PR: ");
    Serial.print(pulsosR);
    Serial.print(" | Prom: ");
    Serial.println(promedio);
    
    if (promedio >= PULSOS_GIRO_90) {
      break;
    }
    delay(10);
  }
  
  stopMotors();
  delay(300);
  
  motorL.setSpeed(pwmL);
  motorR.setSpeed(pwmR);
  
  dir = (Direction)((dir + 1) % 4);
  Serial.print("Nueva dirección: ");
  Serial.println(dir);
  Serial.println("------------------------------------");
}

void girarIzquierdaCalibrado() {
  stopMotors();
  delay(300);
  
  // Verificar seguridad antes de girar
  waitForClearPath();
  
  noInterrupts();
  cntL = 0;
  cntR = 0;
  interrupts();
  
  Serial.println("------------------------------------");
  Serial.println(">>> RESET de pulsos antes del giro");
  Serial.print("Iniciando giro IZQUIERDA 90°. Objetivo: ");
  Serial.print(PULSOS_GIRO_90);
  Serial.println(" pulsos");
  Serial.println("🟢 ESTADO: LIBRE - Girando izquierda");
  
  motorL.setSpeed(55);
  motorR.setSpeed(77);
  motorL.backward();
  motorR.forward();
  
  int velActual = VELOCIDAD_GIRO;
  const int PULSOS_DECEL_GIRO = 2;
  
  while (true) {
    // Verificar seguridad durante giro
    if (checkSafetyZone()) {
      Serial.println("🔴 ESTADO: BLOQUEADO durante giro");
      stopMotors();
      waitForClearPath();
      Serial.println("🟢 ESTADO: LIBRE - Continuando giro");
      motorL.setSpeed(velActual);
      motorR.setSpeed(velActual);
      motorL.backward();
      motorR.forward();
    }
    
    noInterrupts();
    unsigned int pulsosL = cntL;
    unsigned int pulsosR = cntR;
    interrupts();
    
    int promedio = (pulsosL + pulsosR) / 2;
    
    if (promedio >= (PULSOS_GIRO_90 - PULSOS_DECEL_GIRO)) {
      int restantes = PULSOS_GIRO_90 - promedio;
      if (restantes > 0) {
        float factor = (float)restantes / PULSOS_DECEL_GIRO;
        velActual = 51 + (int)((VELOCIDAD_GIRO - 51) * factor);
        motorL.setSpeed(velActual);
        motorR.setSpeed(velActual);
      }
    }
    
    Serial.print("🟢 LIBRE | PL: ");
    Serial.print(pulsosL);
    Serial.print(" | PR: ");
    Serial.print(pulsosR);
    Serial.print(" | Prom: ");
    Serial.println(promedio);
    
    if (promedio >= PULSOS_GIRO_90) {
      break;
    }
    delay(10);
  }
  
  stopMotors();
  delay(300);
  
  motorL.setSpeed(pwmL);
  motorR.setSpeed(pwmR);
  
  dir = (Direction)((dir + 3) % 4);
  Serial.print("Nueva dirección: ");
  Serial.println(dir);
  Serial.println("------------------------------------");
}

// ========== NAVEGACIÓN ==========

void navigateToTarget() {
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║   NAVEGACIÓN 3x3 CON SENSOR RADAR     ║");
  Serial.println("║   Detención automática a 40cm         ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println();
  
  int maxSteps = 100;
  int steps = 0;
  
  while ((posX != targetX || posY != targetY) && steps < maxSteps) {
    steps++;
    
    Serial.println("╔════════════════════════════════════════╗");
    Serial.print("║ PASO ");
    Serial.print(steps);
    Serial.println("                                  ║");
    Serial.print("║ Posición: (");
    Serial.print(posX);
    Serial.print(", ");
    Serial.print(posY);
    Serial.println(")                         ║");
    Serial.print("║ Dirección: ");
    const char* dirNames[] = {"NORTE", "ESTE", "SUR", "OESTE"};
    Serial.print(dirNames[dir]);
    Serial.println("                         ║");
    Serial.println("╚════════════════════════════════════════╝");
    
    bool moved = false;
    
    if (canMoveForward()) {
      Serial.println(">>> AVANZANDO 1.7m");
      moveForwardCalibratedDistance();
      moved = true;
    } else {
      Serial.println(">>> OBSTÁCULO DETECTADO");
      
      girarDerechaCalibrado();
      if (canMoveForward()) {
        Serial.println(">>> Avanzando después de girar derecha");
        moveForwardCalibratedDistance();
        moved = true;
      } else {
        girarIzquierdaCalibrado();
        girarIzquierdaCalibrado();
        if (canMoveForward()) {
          Serial.println(">>> Avanzando después de girar izquierda");
          moveForwardCalibratedDistance();
          moved = true;
        } else {
          girarIzquierdaCalibrado();
          if (canMoveForward()) {
            Serial.println(">>> Avanzando en última dirección");
            moveForwardCalibratedDistance();
            moved = true;
          }
        }
      }
    }
    
    if (!moved) {
      Serial.println("¡ERROR! No hay camino posible");
      break;
    }
    
    delay(500);
  }
  
  Serial.println();
  Serial.println("╔════════════════════════════════════════╗");
  if (posX == targetX && posY == targetY) {
    Serial.println("║    ¡¡¡ OBJETIVO ALCANZADO !!!         ║");
    Serial.println("║         META COMPLETADA               ║");
  } else {
    Serial.println("║    No se pudo alcanzar el objetivo   ║");
  }
  Serial.println("╚════════════════════════════════════════╝");
  stopMotors();
}

// ========== SETUP Y LOOP ==========

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║   ROBOT NAVEGACIÓN 3x3 + RADAR        ║");
  Serial.println("║   HLK-LD2420 - Detención a 40cm       ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println();
  
  // Configuración del Radar
  pinMode(RADAR_OT2, INPUT);
  RadarSerial.begin(115200, SERIAL_8N1, RADAR_RX, RADAR_TX);
  Serial.println("✓ Radar HLK-LD2420 iniciado");
  
  // IMPORTANTE: Configurar el radar para detectar solo en rango corto
  delay(1000);
  Serial.println("⚙  Configurando gates del radar...");
  Serial.println("   NOTA: Debes configurar el radar con el software HLK-LD2420");
  Serial.println("   para que solo use Gates 0-2 (0-60cm aprox)");
  
  // Configuración de sensores Hall
  pinMode(HALL_L, INPUT_PULLUP);
  pinMode(HALL_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_L), onHallL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_R), onHallR, CHANGE);
  Serial.println("✓ Sensores Hall configurados");
  motorL.setSpeed(pwmL);
  motorR.setSpeed(pwmR);
  motorL.runFor(5000, L298N::Direction::FORWARD);
  motorR.runFor(5000, L298N::Direction::FORWARD);
}

void loop() {
  // Mantener lectura del radar incluso después de completar navegación
  // readRadarData();
  delay(100);
}