int MotorDer1=13; //El pin 13 de arduino se conecta con el pin In1 del L298N 
int MotorDer2=11; //El pin 11 de arduino se conecta con el pin In2 del L298N 
int MotorIzq1=9; //El pin 9 de arduino se conecta con el pin In3 del L298N 
int MotorIzq2=8; //El pin 8 de arduino se conecta con el pin In4 del L298N 
int PWM_Derecho=12; //El pin 12 de arduino se conecta con el pin EnA del L298N 
int PWM_Izquierdo=10; //El pin 10 de arduino se conecta con el pin EnB del L298N 

int ldrPinizq = A0;              // LDR pin izq
int ldrizq = 0;               // Valor LDR izq
int ldrPinder = A1;              // LDR pin der
int ldrder = 0;               // Valor LDR der
int error= 0;
int caso=0;
double pwmder=100, pwmizq=100; //PWM derecho e izquierdo
unsigned long currentTime, previousTime=0, elapsedTime; //Tiempo actual, tiempo anterior, tiempo transcurrido


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
  
  ldrizq = kalmanizq(analogRead(ldrPinizq)+50);    // Read the analog value of the LDR
  ldrder = kalmander(analogRead(ldrPinder));    // Read the analog value of the LDR 
  error = ldrder-ldrizq;
  Serial2.println(String(ldrizq) + " " + String(ldrder)+ " " + String(error)+ " " + String(caso));

  delay(100);                     // Pause 100ms
  
    if(pwmder>220){ //Saturación de avance
      pwmder=220;
    }
    if(pwmder<-80){ //Saturación de retroceso
      pwmder=-80;
    }
    if(pwmizq>220){ //Saturación de avance
      pwmizq=220;
    }
    if(pwmizq<-80){ //Saturación de retroceso
      pwmizq=-80;
    }

    switch(caso){
    case 0:
      para();
      Serial2.println(String(ldrizq) + " " + String(ldrder)+ " " + String(error)+ " " + String(caso));
      if(error>25){
        caso=1;
        break;
      }
      if(error>150){
        caso=2;
        break;
      }
      if(error<-80){
        caso=3;
        break;
      }  
      else{
        caso=0;
      }
      break;
    case 1: 
      avanza(100, 100); //Avanza
      Serial2.println(String(ldrizq) + " " + String(ldrder)+ " " + String(error)+ " " + String(caso));
      if(error<30){
        caso=0;
        break;
      }
      if(error>250){
        caso=2;
        break;
      }
      if(error<-80){
        caso=3;
        break;
      }
      
      break;
    case 2:
      pivote_der(); //Pivota a la derecha
      Serial2.println(String(ldrizq) + " " + String(ldrder)+ " " + String(error)+ " " + String(caso));

      if(error<250){
        caso=1;
        break;
      }
      break;
    case 3:
     pivote_izq(); //Pivota a la derecha
     Serial2.println(String(ldrizq) + " " + String(ldrder)+ " " + String(error)+ " " + String(caso));

      if(error>0 && error<250){
        caso=1;
        break;
      }
      if(error>250){
        caso=2;
        break;
      }
      break;
    }
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
  analogWrite(PWM_Derecho,80);
  analogWrite(PWM_Izquierdo,0);
  }

void pivote_der(){ //Orden de pivotar a la derecha
  digitalWrite(MotorDer1,LOW);   
  digitalWrite(MotorDer2,HIGH);
  digitalWrite(MotorIzq1,HIGH);   
  digitalWrite(MotorIzq2,LOW);
  analogWrite(PWM_Derecho,0);
  analogWrite(PWM_Izquierdo,80);
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
