// Configuramos los pines que vamos a usar
#include <Ultrasonic.h>
int MotorDer1 = 13; //El pin 13 de arduino se conecta con el pin In1 del L298N
int MotorDer2 = 11; //El pin 11 de arduino se conecta con el pin In2 del L298N
int MotorIzq1 = 9; //El pin 9 de arduino se conecta con el pin In3 del L298N
int MotorIzq2 = 8; //El pin 8 de arduino se conecta con el pin In4 del L298N
int PWM_Derecho = 12; //El pin 12 de arduino se conecta con el pin EnA del L298N
int PWM_Izquierdo = 10; //El pin 10 de arduino se conecta con el pin EnB del L298N

double  distanciaderbruto, distanciaizqbruto, distanciader, distanciaizq, distanciamed; //Distancia que miden los ultrasonidos
int ref = 0, refbt; //Referencia
double pwmder = 0, pwmizq = 0; //PWM derecho e izquierdo
unsigned long currentTime, previousTime = 0, elapsedTime; //Tiempo actual, tiempo anterior, tiempo transcurrido
double errorder, errorizq, rateErrorder, rateErrorizq, cumErrorder, cumErrorizq, lastErrorder = 0, lastErrorizq = 0 ; //Error, err derivativo, err integral, error anterior
double kp = 0.8, ki = 0 , kd = 40; //Constantes del PID

Ultrasonic ultrader(7, 6); //Ultrasonido der
Ultrasonic ultraiz(5, 4); //Ultrasonido izq

void setup() {
  //Configuramos los pines como salida
  pinMode(MotorDer1, OUTPUT);
  pinMode(MotorDer2, OUTPUT);
  pinMode(MotorIzq1, OUTPUT);
  pinMode(MotorIzq2, OUTPUT);
  pinMode(PWM_Derecho, OUTPUT);
  pinMode(PWM_Izquierdo, OUTPUT);
  //Arrancamos los puertos serie
  Serial.begin (9600);
  Serial2.begin (38400);
}

void loop() {

  currentTime = millis(); //Función que mantiene la cuenta del tiempo
  elapsedTime = currentTime - previousTime; //Cálculo del tiempo transcurrido

  if (Serial2.available() > 0) {
    refbt = Serial2.parseInt();
    if (refbt != 0) {
      ref = refbt;
    }
  }

  distanciaderbruto = ultrader.read(CM) - 1;
  distanciaizqbruto = ultraiz.read(CM);
  distanciader = kalman(distanciaderbruto);
  distanciaizq = kalman(distanciaizqbruto);

  if (distanciader > 200 || distanciaizq > 200) {
    distanciader = 200;
    distanciaizq = 200;
  }

  errorder = distanciader - (double)ref; //Error proporcional
  errorizq = distanciaizq - (double)ref;

  cumErrorder += errorder * (double)elapsedTime; //Error integral
  cumErrorizq += errorizq * (double)elapsedTime;

  rateErrorder = (errorder - lastErrorder) / (double)elapsedTime; //Error derivativo
  rateErrorizq = (errorizq - lastErrorizq) / (double)elapsedTime;

  pwmder = kp * errorder + ki * cumErrorder + kd * rateErrorder; //Salida del controlador
  pwmizq = kp * errorizq + ki * cumErrorizq + kd * rateErrorizq;


  if (errorder > 0 || errorizq > 0) { //Arreglo de zona muerta
    pwmder = pwmder + 70;
    pwmizq = pwmizq + 70;
  }
  else if (errorder < 0 || errorizq < 0) {
    pwmder = pwmder-70;
    pwmizq = pwmizq-70;
  }

  if (pwmder > 140 || pwmizq > 140) { //Saturación de avance
    pwmder = 140;
    pwmizq = 140;
  }
  if (pwmder < -80 || pwmizq < -80) { //Saturación de retroceso
    pwmder = -80;
    pwmizq = -80;
  }
  if (pwmder > 0 && ref != 0) {
    avanzader(abs(pwmder));
  }
  else if (pwmder < 0 && ref != 0) {
    atrasder(abs(pwmder));
  }
  if (pwmizq > 0 && ref != 0) {
    avanzaizq(abs(pwmizq));
  }
  else if (pwmizq < 0 && ref != 0) {
    atrasizq(abs(pwmizq));
  }

  if (abs(errorder) < 1) {
    parader();
    errorder=0;
    rateErrorder=0;
    cumErrorder=0;
    lastErrorder=0;
    pwmder=0;
  }
  if (abs(errorizq) < 1) {
    paraizq();
    errorizq=0;
    rateErrorizq=0;
    cumErrorizq=0;
    lastErrorizq=0;
    pwmizq=0;
  }


  Serial.print(errorder);
  Serial.print("  ");
  Serial.print(errorizq);
  Serial.print("  ");
  Serial.print(ref);
  Serial.print("  ");
  Serial.print(distanciaizq);
  Serial.print("  ");
  Serial.print(distanciader);
  Serial.print("  ");
  Serial.print(pwmizq);
  Serial.print("  ");
  Serial.println(pwmder);


  previousTime = currentTime;
  lastErrorder = errorder;
  lastErrorizq = errorizq;


}

//Funciones para ejecutar distintos movimientos

void parader() { //Orden de parar
  digitalWrite(MotorDer1, LOW);
  digitalWrite(MotorDer2, LOW);
};
void paraizq() { //Orden de parar
  digitalWrite(MotorIzq1, LOW);
  digitalWrite(MotorIzq2, LOW);
};

void avanzader(double pwmder) { //Orden de avanzar rueda derecha
  digitalWrite(MotorDer1, HIGH);
  digitalWrite(MotorDer2, LOW);
  analogWrite(PWM_Derecho, pwmder + 5);
}

void avanzaizq(double pwmizq) { //Orden de avanzar rueda izquierda
  digitalWrite(MotorIzq1, HIGH);
  digitalWrite(MotorIzq2, LOW);
  analogWrite(PWM_Izquierdo, pwmizq);
}


void atrasder(double pwmder) { //Orden para retroceder rueda derecha
  digitalWrite(MotorDer1, LOW);
  digitalWrite(MotorDer2, HIGH);
  analogWrite(PWM_Derecho, pwmder+5);
}

void atrasizq(double pwmizq) { //Orden para retroceder rueda derecha
  digitalWrite(MotorIzq1, LOW);
  digitalWrite(MotorIzq2, HIGH);
  analogWrite(PWM_Izquierdo, pwmizq);
}

double kalman(double U) {
  static const double R = 10;
  static const double H = 1.00;
  static double Q = 5;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  K = P * H / (H * P * H + R);
  U_hat += + K * (U - H * U_hat);
  P = (1 - K * H) * P + Q;
  return U_hat;
}
