#include <Servo.h>  //servo library
Servo myservo;      // create servo object to control servo

// Configurazioni motore
#define carSpeed 130       // Velocità base
#define carSpeedTurn 180   // Velocità per svolte

int SensorBDX = A1; // Sensore IR dietro destra
int SensorBSX = A2; // Sensore IR dietro sinistra

int Sensor1; //sensore1 per follow linea 
int Sensor2; //sensore2 per follow linea 
int Sensor3; //sensore3 per follow linea 
int Sensor4; //sensore4 per follow linea 

#define TRIGGER_PIN 12
#define ECHO_LEFT A4
#define ECHO_FRONT_LEFT 13
#define ECHO_FRONT_RIGHT A5
#define ECHO_RIGHT A3

#define ENA 5
#define ENB 6
#define IN1 3
#define IN2 4
#define IN3 2
#define IN4 7

unsigned long previousMillis = 0;
const long interval = 200; // Intervallo di lettura non bloccante

void setup() {
  myservo.attach(A0, 700, 2400);  // Servo per telecamera o future espansioni

  // Configurazione pin sensori ultrasuoni
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(ECHO_FRONT_LEFT, INPUT);
  pinMode(ECHO_FRONT_RIGHT, INPUT);
  pinMode(ECHO_RIGHT, INPUT);

  //gestione motori ruote
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  //sensori per seguire la linea
  pinMode(8, INPUT); 
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);

  myservo.write(115);  // Posizione iniziale del servo
  Serial.begin(115200);
}

float readDistance(int triggerPin, int echoPin) {
  // Invio impulso Trigger
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Lettura durata del segnale Echo
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout 30 ms
  if (duration == 0) return -1; // Nessuna lettura valida
  return duration * 0.034 / 2;  // Conversione in cm
}

void moveForward() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Avanti");
}

void turnLeft() {
  analogWrite(ENA, carSpeedTurn);
  analogWrite(ENB, carSpeedTurn);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Sinistra");
}

void turnRight() {
  analogWrite(ENA, carSpeedTurn);
  analogWrite(ENB, carSpeedTurn);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Destra");
}

void moveBackward() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Indietro");
}

void stopMotors() {
  analogWrite(ENA, LOW);
  analogWrite(ENB, LOW);
  Serial.println("Stop!");
}

void loop() {
  static unsigned long lastSensorRead = 0;
  static float distanceLeft = -1, distanceFrontLeft = -1, distanceFrontRight = -1, distanceRight = -1;

  unsigned long currentMillis = millis();

  // Leggi i sensori solo ogni 'interval' millisecondi
  if (currentMillis - lastSensorRead >= interval) {
    lastSensorRead = currentMillis;
    
    //lettura sensori per ricerca linea
    Sensor1 = digitalRead(8);//IN1
    Sensor2 = digitalRead(9);//IN2
    Sensor3 = digitalRead(10);//IN3
    Sensor4 = digitalRead(11);//IN4

    // Letture sensori ultrasuoni, con leggeri ritardi per stabilizzare
    delay(20);
    distanceLeft = readDistance(TRIGGER_PIN, ECHO_LEFT);
    delay(20);
    distanceFrontLeft = readDistance(TRIGGER_PIN, ECHO_FRONT_LEFT);
    delay(20);
    distanceFrontRight = readDistance(TRIGGER_PIN, ECHO_FRONT_RIGHT);
    delay(20);
    distanceRight = readDistance(TRIGGER_PIN, ECHO_RIGHT);
    delay(20);

    Serial.print("Left: "); Serial.print(distanceLeft); Serial.print(" cm, ");
    Serial.print("Front Left: "); Serial.print(distanceFrontLeft); Serial.print(" cm, ");
    Serial.print("Front Right: "); Serial.print(distanceFrontRight); Serial.print(" cm, ");
    Serial.print("Right: "); Serial.print(distanceRight); Serial.println(" cm");
  }

  //codice per gestione inseguimento linea
  if(Sensor4 == LOW && Sensor3 == LOW && Sensor2 == LOW && Sensor1 == LOW){
    moveForward();    
  }else if(Sensor4 == HIGH && Sensor3 == HIGH && Sensor2 == HIGH && Sensor1 == HIGH){
    stopMotors();    
  }else if(Sensor4 == HIGH && Sensor3 == LOW && Sensor2 == LOW && Sensor1 == LOW){
    turnLeft();
  }else if(Sensor4 == LOW && Sensor3 == HIGH && Sensor2 == LOW && Sensor1 == LOW){
    turnLeft();
  }else if(Sensor4 == LOW && Sensor3 == LOW && Sensor2 == HIGH && Sensor1 == LOW){
    turnRight();
  }else if(Sensor4 == LOW && Sensor3 == LOW && Sensor2 == LOW && Sensor1 == HIGH){
    turnRight();
  }
  //se sto seguendo la linea gestisci in modo diverso il controllo dei sensori di distanza (da implementare)

  // Definisci la soglia di sicurezza
  const float thresholdFront = 20.0;
  const float thresholdLat = 5.0;

  // Flag di ostacoli
  bool frontLeftBlocked  = (distanceFrontLeft  < thresholdFront && distanceFrontLeft  > 0);
  bool frontRightBlocked = (distanceFrontRight < thresholdFront && distanceFrontRight > 0);
  bool leftBlocked       = (distanceLeft       < thresholdLat && distanceLeft       > 0);
  bool rightBlocked      = (distanceRight      < thresholdLat && distanceRight      > 0);

  // 1. Se entrambi i frontali sono bloccati, vai indietro
  if (frontLeftBlocked && frontRightBlocked) {
    moveBackward();
    return;
  }

  // 2. Se solo il front-left è bloccato, giriamo a destra se possibile
  if (frontLeftBlocked && !frontRightBlocked) {
    if (!rightBlocked) {
      turnRight();
    } else {
      // se anche il lato destro è bloccato, facciamo retromarcia
      moveBackward();
    }
    return;
  }

  // 3. Se solo il front-right è bloccato, giriamo a sinistra se possibile
  if (frontRightBlocked && !frontLeftBlocked) {
    if (!leftBlocked) {
      turnLeft();
    } else {
      // se anche il lato sinistro è bloccato, facciamo retromarcia
      moveBackward();
    }
    return;
  }

  // 4. Altrimenti, se non ci sono ostacoli frontali,
  //    controlliamo i lati (left, right).
  if (!frontLeftBlocked && !frontRightBlocked) {
    // Se è bloccato a sinistra, gira leggermente a destra
    if (leftBlocked && !rightBlocked) {
      turnRight();
      return;
    }
    // Se è bloccato a destra, gira leggermente a sinistra
    if (!leftBlocked && rightBlocked) {
      turnLeft();
      return;
    }
    // Se entrambi i lati sono liberi o entrambi bloccati, muoviti in avanti
    // (nel caso entrambi bloccati, potresti decidere una manovra particolare)
    if (!leftBlocked && !rightBlocked) {
      moveForward();
    } else {
      // entrambi i lati bloccati => retromarcia
      moveBackward();
    }
    return;
  }
}
