#include <Arduino.h>
#include <QTRSensors.h>

#define NUM_SENSORS 8
uint8_t qtrPins[NUM_SENSORS] = {36, 39, 34, 35, 32, 33, 25, 26};
QTRSensors qtr;

#define BIN1 18
#define BIN2 19
#define AIN2 16
#define AIN1 17
#define PWMB 21
#define PWMA 4

#define LED_CALIBRACION 22
#define BOTON_CALIBRAR  23

#define PWM_FREQ  20000
#define PWM_RES   8
#define LINEA_NEGRA false

const int SETPOINT = 3500; // Centro de 8 sensores (0 a 7000)
float Kp = 0.32f;//0.31f
float Kd = 0.008f;//0.006f

float lastError = 0;
//float derivFiltrado = 0;
//const float ALPHA_D = 0.3f;

const int MAX_PWM       = 255;
const int VELOCIDAD_MAX = 255;
const int VELOCIDAD_MIN = 55;

unsigned long ultimoLoopUs = 0;

enum EstadoRobot { ESTADO_INICIAL, ESTADO_CALIBRANDO, ESTADO_LISTO, ESTADO_SIGUIENDO };
EstadoRobot estado = ESTADO_INICIAL;

void motorIzquierdo(int vel) {
  if (vel >= 0) { 
    digitalWrite(AIN1, LOW); 
  digitalWrite(AIN2, HIGH);
   }
  else { 
    digitalWrite(AIN1, HIGH);
     digitalWrite(AIN2, LOW);
      vel = -vel; }
  ledcWrite(PWMA, constrain(vel, 0, MAX_PWM));
}

void motorDerecho(int vel) {
  if (vel >= 0) {
     digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH); 
      }
  else { 
    digitalWrite(BIN1, HIGH);
     digitalWrite(BIN2, LOW);
      vel = -vel;
       }
  ledcWrite(PWMB, constrain(vel, 0, MAX_PWM));
}

void motoresParar() {
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
}


void calibrar() {
  estado = ESTADO_CALIBRANDO;
  unsigned long tiempoInicio = millis();

  while (millis() - tiempoInicio < 12000) {
    qtr.calibrate();
    digitalWrite(LED_CALIBRACION, (millis() / 100) % 2); // Parpadeo
  }
  
  digitalWrite(LED_CALIBRACION, LOW);
  estado = ESTADO_LISTO;
  Serial.println("Calibración completa.");
}

void manejarBoton() {
  static bool presionadoPrev = HIGH;
  static unsigned long tiempoPresionado = 0;
  bool presionadoAct = digitalRead(BOTON_CALIBRAR);

  if (presionadoPrev == HIGH && presionadoAct == LOW) {
    tiempoPresionado = millis();
  }

  if (presionadoPrev == LOW && presionadoAct == HIGH) {
    unsigned long duracion = millis() - tiempoPresionado;

    if (duracion > 2000) { // +2s: Calibrar
      calibrar();
    } else if (duracion > 50) { // Click corto: Arrancar/Parar
      if (estado == ESTADO_LISTO) {
        estado = ESTADO_SIGUIENDO;
        ultimoLoopUs = micros();
        Serial.println("Robot Iniciado");
      } else if (estado == ESTADO_SIGUIENDO) {
        motoresParar();
        estado = ESTADO_LISTO;
        Serial.println("Robot Detenido");
      }
    }
  }
  presionadoPrev = presionadoAct;
}

void seguirLinea() {
  unsigned long ahoraUs = micros();
  float dt = (ahoraUs - ultimoLoopUs) / 1000000.0f;
  ultimoLoopUs = ahoraUs;
  if (dt > 0.1f) dt = 0.1f;

  uint16_t sensorValues[NUM_SENSORS];
  uint16_t position = LINEA_NEGRA ? qtr.readLineBlack(sensorValues) : qtr.readLineWhite(sensorValues);

  float error = (float)SETPOINT - (float)position;
  
  // PID (PD)
  float Errorderivativo = (error - lastError) / dt;
  //derivFiltrado = ALPHA_D * derivFiltrado + (1.0f - ALPHA_D) * derivRaw;
  float U = (Kp * error) + (Kd * Errorderivativo);
 lastError = error;
U = constrain(U, -255, 255);

int velIzq = VELOCIDAD_BASE + U;
int velDer = VELOCIDAD_BASE - U;
 
  motorIzquierdo(velIzq);
  motorDerecho(velDer);
}

void setup() {
  Serial.begin(115200);

  // Configuración Pines Motores
  pinMode(AIN1, OUTPUT);
   pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
   pinMode(BIN2, OUTPUT);
  
  // Configuración PWM moderna ESP32
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);

  pinMode(LED_CALIBRACION, OUTPUT);
  pinMode(BOTON_CALIBRAR, INPUT_PULLUP);

  // Sensores
  qtr.setTypeAnalog();
  qtr.setSensorPins(qtrPins, NUM_SENSORS);

  Serial.println("Sistema iniciado. ESP32 listo.");
}

void loop() {
  manejarBoton();

  if (estado == ESTADO_SIGUIENDO) {
    seguirLinea();
  } else {
    motoresParar();
  }
}