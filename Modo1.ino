// Configuramos los pines que vamos a usar 
#include <Ultrasonic.h>
int MotorDer1=13; //El pin 13 de arduino se conecta con el pin In1 del L298N 
int MotorDer2=11; //El pin 11 de arduino se conecta con el pin In2 del L298N 
int MotorIzq1=9; //El pin 9 de arduino se conecta con el pin In3 del L298N 
int MotorIzq2=8; //El pin 8 de arduino se conecta con el pin In4 del L298N 
int PWM_Derecho=12; //El pin 12 de arduino se conecta con el pin EnA del L298N 
int PWM_Izquierdo=10; //El pin 10 de arduino se conecta con el pin EnB del L298N 

double  distanciader, distanciaizq, distanciamed; //Distancia que miden los ultrasonidos
int ref, refbt; //Referencia
double pwmder=0, pwmizq=0; //PWM derecho e izquierdo
unsigned long currentTime, previousTime=0, elapsedTime; //Tiempo actual, tiempo anterior, tiempo transcurrido
double error, rateError, cumError, lastError=0; //Error, err derivativo, err integral, error anterior
double kp=0.7, ki=0 ,kd=0.1; //Constantes del PID

Ultrasonic ultrader(7,6); //Ultrasonido der
Ultrasonic ultraiz(5,4); //Ultrasonido izq

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

  if(Serial2.available()>0){
    refbt=Serial2.parseInt();
      if(refbt!=0){
        ref=refbt;       
      }
    }
    
    distanciader = ultrader.read(CM)-1; //para este modo basico usamos ambos sensores aunque no nos haga falta
    distanciaizq = ultraiz.read(CM)+1;
    if(distanciader>200 || distanciaizq>200){
      distanciader=200;
      distanciaizq=200;
    }
    distanciamed=(distanciaizq+distanciader)/2;
    error = distanciamed - (double)ref; //Error proporcional
    cumError += error * (double)elapsedTime; //Error integral
    rateError = error * (error-lastError)/(double)elapsedTime; //Error derivativo

    pwmder = kp*error + ki*cumError + kd*rateError;

    if(error>0){ //Arreglo de zona muerta
      pwmder=pwmder+70;
    }
    else if(error<0){
      pwmder=pwmder-70;
    }

    if(pwmder>140){ //Saturación de avance
      pwmder=140;
    }
    if(pwmder<-80){ //Saturación de retroceso
      pwmder=-80;
    }
    if(pwmder>0 && ref!=0 && error>2){
      avanza(abs(pwmder),abs(pwmder));
    }
    if(pwmder<0 && ref!=0 && error<-2){
      atras(abs(pwmder),abs(pwmder));
    }
    if (error < 2 && error > -2){ 
    para();
    error=0;
    rateError=0;
    cumError=0;
    lastError=0;
    }
    Serial.print(ref);
    Serial.print("  ");  
    Serial.print(distanciamed);
    Serial.print("  ");
    Serial.println(pwmder);
    previousTime = currentTime;
    lastError = error;
  

}

//Funciones para ejecutar distintos movimientos

void para(){ //Orden de parar
  digitalWrite(MotorDer1,LOW);   
  digitalWrite(MotorDer2,LOW);
  digitalWrite(MotorIzq1,LOW);   
  digitalWrite(MotorIzq2,LOW);
  };


void avanza(double pwmizq, double pwmder){ //Orden de avanzar
  digitalWrite(MotorDer1,HIGH);   
  digitalWrite(MotorDer2,LOW);
  digitalWrite(MotorIzq1,HIGH);   
  digitalWrite(MotorIzq2,LOW);
  analogWrite(PWM_Derecho,pwmder+10);
  analogWrite(PWM_Izquierdo,pwmizq);
  }

void pivote_izq(){ //Orden de pivotar a la izquierda
  digitalWrite(MotorDer1,HIGH);   
  digitalWrite(MotorDer2,LOW);
  digitalWrite(MotorIzq1,LOW);   
  digitalWrite(MotorIzq2,HIGH);
  analogWrite(PWM_Derecho,100);
  analogWrite(PWM_Izquierdo,100);
  }

void pivote_der(){ //Orden de pivotar a la derecha
  digitalWrite(MotorDer1,LOW);   
  digitalWrite(MotorDer2,HIGH);
  digitalWrite(MotorIzq1,HIGH);   
  digitalWrite(MotorIzq2,LOW);
  analogWrite(PWM_Derecho,100);
  analogWrite(PWM_Izquierdo,100);
  }

void atras(double pwmizq, double pwmder){ //Orden para retroceder
  digitalWrite(MotorDer1,LOW);   
  digitalWrite(MotorDer2,HIGH);
  digitalWrite(MotorIzq1,LOW);   
  digitalWrite(MotorIzq2,HIGH);
  analogWrite(PWM_Derecho,pwmder);
  analogWrite(PWM_Izquierdo,pwmizq);
  }
