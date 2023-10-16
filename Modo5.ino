int MotorDer1 = 13; //El pin 13 de arduino se conecta con el pin In1 del L298N
int MotorDer2 = 11; //El pin 11 de arduino se conecta con el pin In2 del L298N
int MotorIzq1 = 9; //El pin 9 de arduino se conecta con el pin In3 del L298N
int MotorIzq2 = 8; //El pin 8 de arduino se conecta con el pin In4 del L298N
int PWM_Derecho = 12; //El pin 12 de arduino se conecta con el pin EnA del L298N
int PWM_Izquierdo = 10; //El pin 10 de arduino se conecta con el pin EnB del L298N
int ENC_IZ = 21; //Encoder izquierdo
int ENC_DE = 20; //Encoder derecho


int pwm_iz = 100, pwm_de = 100, K = 1, error_rpm_iz = 0, error_rpm_de = 0, refbt;
float rpm_ref = 100;
unsigned long tiempo = 0, tiempo_f = 0, muestreo_ant = 0, muestreo = 0;
long int tick_iz, tick_de;
float rpm_iz, rpm_de;
void setup() {

  //Configuramos los pines como salida
  pinMode(MotorDer1, OUTPUT);
  pinMode(MotorDer2, OUTPUT);
  pinMode(MotorIzq1, OUTPUT);
  pinMode(MotorIzq2, OUTPUT);
  pinMode(PWM_Derecho, OUTPUT);
  pinMode(PWM_Izquierdo, OUTPUT);

  //Pines de interrupción de los encoders
  attachInterrupt(digitalPinToInterrupt(ENC_IZ), countTicks_iz, CHANGE); //Interrupción encoder izquierdo
  attachInterrupt(digitalPinToInterrupt(ENC_DE), countTicks_de, CHANGE); //Interrupción encoder derecho

  //Arrancamos los puertos serie
  Serial.begin (9600);
  Serial2.begin (38400);
}

void loop() {
  muestreo = millis();
  if (Serial.available() > 0) {
    refbt = Serial.parseInt();
    if (refbt != 0) {
      rpm_ref = refbt;
    }
  }
  if (muestreo - muestreo_ant >= 100) {
    //Calculo de los rpm de cada rueda, cada 100ms///// 374 es la resolucion de los enconders///la formula de los rpm es la siguiente
    //rpm=float((60*pulsos)/resolucion) para un tiempo de calculo de 1s, pero como nosotros estamos calculando cada 100ms, multiplicamos por 600 en vez de por 60
    rpm_iz = ((float)tick_iz / 384) / 0.00166667; //Cálculo de las rpm de las ruedas, midiendo los ticks incrementales
    rpm_de = ((float)tick_de / 384) / 0.00166667; //desde la última medición
    tick_iz = 0;
    tick_de = 0;

    error_rpm_iz = rpm_ref - rpm_iz; //Calculamos el error entre la medicion real de rpm de cada rueda y la referenca
    error_rpm_de = rpm_ref - rpm_de; //Este error se lo restamos o sumamos al pwm de modo que es proporcional al error.

    if (error_rpm_iz > 0) pwm_iz = pwm_iz + 5; //Vamos sumando o restando un offset de 5 a los pwm a medida que se requiere de aumentar o disminuir las rpm
    else pwm_iz = pwm_iz - 5;

    if (error_rpm_de > 0) pwm_de = pwm_de + 5;
    else pwm_de = pwm_de - 5;

    if (abs(pwm_iz) > 220) pwm_iz = 220; //Saturamos los valores pwm del motor
    if (abs(pwm_de) > 220) pwm_de = 220;

    avanza(abs(pwm_iz), abs(pwm_de)); //Avanza
    Serial.println(String(muestreo - muestreo_ant) + " " + String(rpm_iz) + " " + String(rpm_de) + " " + String(rpm_ref) + " " + "5" + " " + String(abs(pwm_iz)) + " " + String(abs(pwm_de)) + " " + String(error_rpm_iz) + " " + String(error_rpm_de));
    muestreo_ant = muestreo;
  }
}
void countTicks_iz() {
  tick_iz++;
}
void countTicks_de() {
  tick_de++;
}

void para() { //Orden de parar
  digitalWrite(MotorDer1, LOW);
  digitalWrite(MotorDer2, LOW);
  digitalWrite(MotorIzq1, LOW);
  digitalWrite(MotorIzq2, LOW);
}


void avanza(double pwmizq, double pwmder) { //Orden de avanzar
  digitalWrite(MotorDer1, HIGH);
  digitalWrite(MotorDer2, LOW);
  analogWrite(PWM_Derecho, pwmder);
  digitalWrite(MotorIzq1, HIGH);
  digitalWrite(MotorIzq2, LOW);
  analogWrite(PWM_Izquierdo, pwmizq + 5);
}

void atras(double pwmizq, double pwmder) { //Orden para retroceder
  digitalWrite(MotorDer1, LOW);
  digitalWrite(MotorDer2, HIGH);
  digitalWrite(MotorIzq1, LOW);
  digitalWrite(MotorIzq2, HIGH);
  analogWrite(PWM_Derecho, pwmder);
  analogWrite(PWM_Izquierdo, pwmizq);
}
