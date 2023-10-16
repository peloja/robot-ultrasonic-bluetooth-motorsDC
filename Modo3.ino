// Configuramos los pines que vamos a usar
#include <Ultrasonic.h>
int MotorDer1 = 13; //El pin 13 de arduino se conecta con el pin In1 del L298N
int MotorDer2 = 11; //El pin 11 de arduino se conecta con el pin In2 del L298N
int MotorIzq1 = 9; //El pin 9 de arduino se conecta con el pin In3 del L298N
int MotorIzq2 = 8; //El pin 8 de arduino se conecta con el pin In4 del L298N
int PWM_Derecho = 12; //El pin 12 de arduino se conecta con el pin EnA del L298N
int PWM_Izquierdo = 10; //El pin 10 de arduino se conecta con el pin EnB del L298N

double  distanciaderbruto, distanciaizqbruto, distanciader, distanciaizq; //Distancia que miden los ultrasonidos
int ref = 0, refbt; //Referencia
//double pwmder = 0, pwmizq = 0; //PWM derecho e izquierdo
int pwm = 0, pwmizq=0, pwmder=0;
unsigned long currentTime, previousTime = 0, elapsedTime; //Tiempo actual, tiempo anterior, tiempo transcurrido
double err, err_i, err_d, err_ant = 0;
double kp = 0.7, ki = 0 , kd = 4.5; //Constantes del PID

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

void loop() 
{
  currentTime = millis(); //Función que mantiene la cuenta del tiempo
  elapsedTime = currentTime - previousTime; //Cálculo del tiempo transcurrido

  //referencia por serial  
  if (Serial2.available() > 0) {
    refbt = Serial2.parseInt();
    if (refbt != 0) {
      ref = refbt;
    }    
  }

  if(elapsedTime >= 50)
  {
    //mido la distancia
    distanciaderbruto = ultrader.read(CM);
    distanciaizqbruto = ultraiz.read(CM);
    distanciader = kalmander(distanciaderbruto);
    distanciaizq = kalmanizq(distanciaizqbruto);
  
    err = (double)distanciaizq - (double)distanciader; //calculo del error relativo entre las dos medidas de los ultrasonidos
    err_i += err * (double)elapsedTime; //error integral
    err_d = err * (err-err_ant) / (double)elapsedTime; // error derivativo
    
    pwm = kp*err + ki*err_i + kd*err_d; //salida del controlador
       
    //Saturación de avance  
    if (pwm > 140) { 
      pwm = 140;
    }

    if(err>=-1.5 && err<=1.5) //tomamos este margen para mantener el robot paralelo
    {
      avanza(83);
      pwmizq=83;
      pwmder=83;
    }
    else if(err<-1.5) //el robot gira a la izquierda    
    {
      if(err>4 || err<-4){
        err=-4;
      }
      giroizq(abs(pwm));
      pwmder=95-abs(pwm);
      pwmizq=83;        
    }    
    else if(err>1.5) // el robot gira a la derecha
    {
      if(err>4 || err<-4){
        err=4;
      }
      giroder(abs(pwm));
      pwmizq=95+abs(pwm);
      pwmder=83;
    }
    
    
   //Serial.println(String(distanciaizq) + " " + String(distanciader)+ " " + String(distanciaizqbruto) + " " + String(distanciaderbruto)+ " " + String(err));
  Serial2.println(String(elapsedTime) + " " + String(distanciaizq) + " " + String(distanciader) + " " + "50" + " " + "3" + " " + pwmizq + " " + pwmder + " " + String(err));
  previousTime = currentTime;
  err_ant = err;        
  }        
        
}


//Funciones para ejecutar distintos movimientos

void para() { //Orden de parar
  digitalWrite(MotorDer1, LOW);
  digitalWrite(MotorDer2, LOW);
  digitalWrite(MotorIzq1, LOW);
  digitalWrite(MotorIzq2, LOW);
}


void avanza(double pwm) { //Orden de avanzar 
  digitalWrite(MotorDer1, HIGH);
  digitalWrite(MotorDer2, LOW);
  analogWrite(PWM_Derecho, pwm+3);
  digitalWrite(MotorIzq1, HIGH);
  digitalWrite(MotorIzq2, LOW);
  analogWrite(PWM_Izquierdo, pwm+3);
}


void giroder (double pwm) {
  
  digitalWrite(MotorDer1, HIGH);
  digitalWrite(MotorDer2, LOW); 
  digitalWrite(MotorIzq1, HIGH); 
  digitalWrite(MotorIzq2, LOW); 
  analogWrite(PWM_Izquierdo, 95+pwm);
  analogWrite(PWM_Derecho, 83);
}

void giroizq (double pwm) {
  
  digitalWrite(MotorDer1, HIGH);
  digitalWrite(MotorDer2, LOW); 
  digitalWrite(MotorIzq1, HIGH); 
  digitalWrite(MotorIzq2, LOW); 
  analogWrite(PWM_Izquierdo, 83);
  analogWrite(PWM_Derecho, 95-pwm);
}



double kalmander(double U) {
  static const double R = 10;
  static const double H = 1.00;
  static double Q = 1;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  K = P * H / (H * P * H + R);
  U_hat += + K * (U - H * U_hat);
  P = (1 - K * H) * P + Q;
  return U_hat;
}

double kalmanizq(double U) {
  static const double R = 10;
  static const double H = 1.00;
  static double Q = 1;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  K = P * H / (H * P * H + R);
  U_hat += + K * (U - H * U_hat);
  P = (1 - K * H) * P + Q;
  return U_hat;
}
