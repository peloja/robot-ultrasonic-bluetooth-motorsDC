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
double pwm_der, pwm_izq, pwm = 0, pwm_eiz, pwm_ede; //PWM derecho e izquierdo
unsigned long currentTime, previousTime = 0, elapsedTime; //Tiempo actual, tiempo anterior, tiempo transcurrido
double err,err_i,err_d, err_ant = 0, err_de, err_iz ; //Error, err derivativo, err integral, error anterior
double Kp = 10, Ki = 0 , Kd = 5, Kp_lineal = 0.8; //Constantes del PID

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

//referencia por serial
  if (Serial2.available() > 0) {
    refbt = Serial2.parseInt();
    if (refbt != 0) {
      ref = refbt;
    }
  }

if(elapsedTime>=100)
{
//calculo las distancias 
  distanciaderbruto = ultrader.read(CM) - 1;
  distanciaizqbruto = ultraiz.read(CM);
  distanciader = kalmander(distanciaderbruto);
  distanciaizq = kalmanizq(distanciaizqbruto);
  
if ((distanciaizq > ref + 2) && (distanciader > ref + 2)) {
      err_iz = (double)distanciader - (double)ref; //Cálculo del error relativo
      err_i += err * (double)elapsedTime; //Error integral
      pwm_eiz = Kp*err_iz + Ki*err_i + Kd*err_d; //Salida del controlador

      if (pwm_eiz > 10){
        pwm_eiz = 10;
      }
      else if (pwm_eiz < -10){
        pwm_eiz = -10;
      }
      
      pwm_izq = 100 - abs(pwm_eiz); //Tomamos un pwm fijo para cada rueda, de 100
      pwm_der = 100;
      giro(pwm_izq,pwm_der);
      Serial2.println(String(elapsedTime) + " " + String(distanciaizq) + " " + String(distanciader) + " " + String(ref) + " " + "4" + " " + String(pwm_izq) + " " + String(pwm_der) + " " + String(err_iz) + " " + String(err_de));

      
    }
    else if ((distanciaizq > ref - 2) && (distanciader > ref - 2)) {
      err_de = (double)distanciaizq - (double)ref;
      err_d = err * (err-err_ant)/(double)elapsedTime; //Error derivativo
      pwm_ede = Kp*err_de + Ki*err_i + Kd*err_d; //Salida del controlador
  
      if (pwm_ede > 10){
        pwm_ede = 10;
      }
      else if (pwm_ede < -10){
        pwm_ede = - 10;
      }
      
      pwm_izq = 100;
      pwm_der = 100 - abs(pwm_ede);
      giro(pwm_izq,pwm_der);
      Serial2.println(String(elapsedTime) + " " + String(distanciaizq) + " " + String(distanciader) + " " + String(ref) + " " + "4" + " " + String(pwm_izq) + " " + String(pwm_der) + " " + String(err_iz) + " " + String(err_de));
    }
    else { //Si se encuetra entre el margen establecido, se aplica el control del modo anterior
      err = (double)distanciaizq - (double)distanciader; //Cálculo del error relativo
      err_i += err * (double)elapsedTime; //Error integral
      err_d = err * (err-err_ant)/(double)elapsedTime; //Error derivativo
      pwm = Kp_lineal*err + Ki*err_i + Kd*err_d; //Salida del controlador

      if (pwm>140){
        pwm = 140; //Saturación de la salida
      }
      
      if (err>=-2 && err<=2) {
        avanza(80);
        Serial2.println(String(elapsedTime) + " " + String(distanciaizq) + " " + String(distanciader) + " " + String(ref) + " " + "4" + " " + "80" + " " + "80" + " " + String(err) + " " + String(err));
      }
       else if(err<-2) //el robot gira a la izquierda    
    {
      
      if(err>4 || err<-4){
        err=-4;
      }
      giroizq(abs(pwm));
        Serial2.println(String(elapsedTime) + " " + String(distanciaizq) + " " + String(distanciader) + " " + String(ref) + " " + "4" + " " + "String(90 - pwm)" + " " + "90" + " " + String(err) + " " + String(err));
                   
    }
           
    else if(err>2) // el robot gira a la derecha
    {
      
      if(err>4 || err<-4){
        err=4;
      }
      giroder(abs(pwm));
        Serial2.println(String(elapsedTime) + " " + String(distanciaizq) + " " + String(distanciader) + " " + String(ref) + " " + "4" + " " + "90" + " " + "String(90 - pwm)" + " " + String(err) + " " + String(err));
    }
    
    //Serial.println(String(distanciaizq) + " " + String(distanciader)+ " " + String(distanciaizqbruto) + " " + String(distanciaderbruto)+ " " + String(err));
    Serial2.println(String(elapsedTime) + " " + String(distanciaizq) + " " + String(distanciader) + " " + "50" + " " + "3" + " " + pwm + " " + pwm + " " + String(err));
  
    previousTime = currentTime;
    err_ant = err;
  }}}

  

//Funciones para ejecutar distintos movimientos

void avanza(double pwm) { //Orden de avanzar 
  digitalWrite(MotorDer1, HIGH);
  digitalWrite(MotorDer2, LOW);
  analogWrite(PWM_Derecho, pwm);
  digitalWrite(MotorIzq1, HIGH);
  digitalWrite(MotorIzq2, LOW);
  analogWrite(PWM_Izquierdo, pwm+5);
}

void giro(int pwm_iz, int pwm_de) { //Alejar o acercar al robot
  
  digitalWrite(MotorDer1, HIGH); 
  digitalWrite(MotorDer2, LOW); 
  digitalWrite(MotorIzq1, HIGH); 
  digitalWrite(MotorIzq2, LOW); 
   
  analogWrite(PWM_Derecho, pwm_de +5); //Motor derecho
  analogWrite(PWM_Izquierdo, pwm_iz ); //Motor izquierdo
}

void giroder (double pwm) {
  
  digitalWrite(MotorDer1, HIGH);
  digitalWrite(MotorDer2, LOW); 
  digitalWrite(MotorIzq1, HIGH); 
  digitalWrite(MotorIzq2, LOW); 
  analogWrite(PWM_Izquierdo, 90);
  analogWrite(PWM_Derecho, 90-pwm);
}

void giroizq (double pwm) {
  
  digitalWrite(MotorDer1, HIGH);
  digitalWrite(MotorDer2, LOW); 
  digitalWrite(MotorIzq1, HIGH); 
  digitalWrite(MotorIzq2, LOW); 
  analogWrite(PWM_Izquierdo, 90-pwm);
  analogWrite(PWM_Derecho, 90);
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
