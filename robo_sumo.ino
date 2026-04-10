#include<Arduino.h>

/*
===========================================================
ROBOT SUMO - ESP32 + FLYSKY + L298N
LOGICA CORREGIDA
===========================================================

CH3 = magnitud / acelerador maestro
- abajo del todo  -> 0 movimiento
- arriba          -> mayor velocidad disponible

CH2 = avance / retroceso (stick derecho eje Y)
- adelante -> avanza
- atras    -> retrocede

CH1 = direccion (stick derecho eje X)
- izquierda -> curva izquierda
- derecha   -> curva derecha

IMPORTANTE:
- Si CH3 esta abajo, el robot NO se mueve
- El stick derecho define el movimiento
- El robot gira por diferencia de velocidades, no por giro en sitio

PINES RC:
- CH1 -> GPIO 34
- CH2 -> GPIO 35
- CH3 -> GPIO 32

PINES L298N:
Motor izquierdo:
- IN1 -> GPIO 18
- IN2 -> GPIO 19
- ENA -> GPIO 23

Motor derecho:
- IN3 -> GPIO 16 (RX2 en algunas placas)
- IN4 -> GPIO 17 (TX2 en algunas placas)
- ENB -> GPIO 5
===========================================================
*/

// =========================
// PINES RECEPTOR
// =========================
constintPIN_RC_CH1 =34;// direccion
constintPIN_RC_CH2 =35;// avance / retroceso
constintPIN_RC_CH3 =32;// magnitud / acelerador

// =========================
// PINES L298N
// =========================
constintPIN_IN1 =18;
constintPIN_IN2 =19;
constintPIN_ENA =23;

constintPIN_IN3 =16;// RX2 en algunas placas
constintPIN_IN4 =17;// TX2 en algunas placas
constintPIN_ENB =5;

// =========================
// PWM
// =========================
constintPWM_FREQ =20000;
constintPWM_RESOLUTION =8;
constintPWM_MAX =255;

// =========================
// RC
// =========================
constintRC_MIN =1000;
constintRC_CENTER =1500;
constintRC_MAX =2000;
constintRC_DEADBAND =40;
constunsignedlongRC_TIMEOUT =30000;

// =========================
// AJUSTES DE CONDUCCION
// =========================

// Velocidad minima util para vencer friccion
constintPWM_MIN_EFECTIVO =70;

// Influencia de la direccion sobre la mezcla
constfloatFACTOR_DIRECCION =0.80;

// Compensacion por diferencias mecanicas
constfloatAJUSTE_MOTOR_IZQ =1.00;
constfloatAJUSTE_MOTOR_DER =1.00;

// Umbral para considerar CH3 en cero real
constintTHROTTLE_CERO_US =1050;

// Debug serial
unsignedlongultimoPrint =0;
constunsignedlongPRINT_INTERVAL_MS =150;

// ===========================================================
// FUNCIONES RC
// ===========================================================
intleerPulsoRC(intpin) {
unsignedlongvalor =pulseIn(pin,HIGH,RC_TIMEOUT);
if (valor==0) {
returnRC_CENTER;
  }
return (int)valor;
}

intnormalizarBidireccional(intpulso) {
// 1000..2000 -> -100..100
if (abs(pulso-RC_CENTER)<=RC_DEADBAND) {
return0;
  }

if (pulso<RC_CENTER) {
pulso =constrain(pulso,RC_MIN,RC_CENTER);
returnmap(pulso,RC_MIN,RC_CENTER,-100,0);
  }else {
pulso =constrain(pulso,RC_CENTER,RC_MAX);
returnmap(pulso,RC_CENTER,RC_MAX,0,100);
  }
}

intnormalizarMagnitud(intpulso) {
// CH3 abajo = 0
// CH3 arriba = 100
pulso =constrain(pulso,RC_MIN,RC_MAX);

if (pulso<=THROTTLE_CERO_US) {
return0;
  }

returnmap(pulso,THROTTLE_CERO_US,RC_MAX,0,100);
}

// ===========================================================
// FUNCIONES MOTORES
// ===========================================================
intaplicarMinimoEfectivo(intvalorPWM) {
if (valorPWM==0)return0;

intmagnitud =abs(valorPWM);
magnitud =map(magnitud,1,255,PWM_MIN_EFECTIVO,255);
magnitud =constrain(magnitud,0,255);

return (valorPWM>0) ?magnitud :-magnitud;
}

voidescribirMotorIzquierdo(intvelocidad) {
velocidad =constrain(velocidad,-255,255);

if (velocidad>0) {
digitalWrite(PIN_IN1,HIGH);
digitalWrite(PIN_IN2,LOW);
ledcWrite(PIN_ENA,velocidad);
  }elseif (velocidad<0) {
digitalWrite(PIN_IN1,LOW);
digitalWrite(PIN_IN2,HIGH);
ledcWrite(PIN_ENA,-velocidad);
  }else {
digitalWrite(PIN_IN1,LOW);
digitalWrite(PIN_IN2,LOW);
ledcWrite(PIN_ENA,0);
  }
}

voidescribirMotorDerecho(intvelocidad) {
velocidad =constrain(velocidad,-255,255);

if (velocidad>0) {
digitalWrite(PIN_IN3,HIGH);
digitalWrite(PIN_IN4,LOW);
ledcWrite(PIN_ENB,velocidad);
  }elseif (velocidad<0) {
digitalWrite(PIN_IN3,LOW);
digitalWrite(PIN_IN4,HIGH);
ledcWrite(PIN_ENB,-velocidad);
  }else {
digitalWrite(PIN_IN3,LOW);
digitalWrite(PIN_IN4,LOW);
ledcWrite(PIN_ENB,0);
  }
}

voiddetenerMotores() {
escribirMotorIzquierdo(0);
escribirMotorDerecho(0);
}

// ===========================================================
// CONTROL DEL ROBOT
// ===========================================================
voidcontrolarRobot(intthrottleCmd,intavanceCmd,intdirCmd) {
/*
    throttleCmd: 0..100      (CH3)
    avanceCmd:   -100..100   (CH2)
    dirCmd:      -100..100   (CH1)

    Idea:
    - CH3 limita la magnitud total
    - CH2 manda avance / retroceso
    - CH1 manda direccion
    - Si throttle=0, no hay movimiento
  */

if (throttleCmd<=0) {
detenerMotores();
return;
  }

// Escalar avance y direccion segun el acelerador maestro
floatavance = ((float)avanceCmd* (float)throttleCmd)/100.0f;
floatgiro   = ((float)dirCmd* (float)throttleCmd)/100.0f;

// Suavizar efecto de la direccion
giro*=FACTOR_DIRECCION;

// Mezcla diferencial
floatvelIzq =avance+giro;
floatvelDer =avance-giro;

velIzq =constrain(velIzq,-100.0f,100.0f);
velDer =constrain(velDer,-100.0f,100.0f);

velIzq*=AJUSTE_MOTOR_IZQ;
velDer*=AJUSTE_MOTOR_DER;

velIzq =constrain(velIzq,-100.0f,100.0f);
velDer =constrain(velDer,-100.0f,100.0f);

intpwmIzq =map((int)abs(velIzq),0,100,0,PWM_MAX);
intpwmDer =map((int)abs(velDer),0,100,0,PWM_MAX);

pwmIzq = (velIzq>=0) ?pwmIzq :-pwmIzq;
pwmDer = (velDer>=0) ?pwmDer :-pwmDer;

pwmIzq =aplicarMinimoEfectivo(pwmIzq);
pwmDer =aplicarMinimoEfectivo(pwmDer);

escribirMotorIzquierdo(pwmIzq);
escribirMotorDerecho(pwmDer);
}

// ===========================================================
// DEBUG
// ===========================================================
voidimprimirEstado(intch1_us,intch2_us,intch3_us,intdirCmd,intavanceCmd,intthrottleCmd) {
Serial.print("CH1(dir): ");
Serial.print(ch1_us);
Serial.print(" | CH2(av): ");
Serial.print(ch2_us);
Serial.print(" | CH3(thr): ");
Serial.print(ch3_us);

Serial.print(" || dirCmd: ");
Serial.print(dirCmd);
Serial.print(" | avanceCmd: ");
Serial.print(avanceCmd);
Serial.print(" | throttleCmd: ");
Serial.println(throttleCmd);
}

// ===========================================================
// SETUP
// ===========================================================
voidsetup() {
Serial.begin(115200);
delay(1000);

pinMode(PIN_RC_CH1,INPUT);
pinMode(PIN_RC_CH2,INPUT);
pinMode(PIN_RC_CH3,INPUT);

pinMode(PIN_IN1,OUTPUT);
pinMode(PIN_IN2,OUTPUT);
pinMode(PIN_IN3,OUTPUT);
pinMode(PIN_IN4,OUTPUT);

ledcAttach(PIN_ENA,PWM_FREQ,PWM_RESOLUTION);
ledcAttach(PIN_ENB,PWM_FREQ,PWM_RESOLUTION);

detenerMotores();

Serial.println("======================================");
Serial.println("Robot sumo iniciado");
Serial.println("CH3 = throttle maestro");
Serial.println("CH2 = avance/reversa");
Serial.println("CH1 = direccion");
Serial.println("Si CH3 esta abajo, no se mueve");
Serial.println("======================================");
}

// ===========================================================
// LOOP
// ===========================================================
voidloop() {
intch1_us =leerPulsoRC(PIN_RC_CH1);
intch2_us =leerPulsoRC(PIN_RC_CH2);
intch3_us =leerPulsoRC(PIN_RC_CH3);

intdirCmd =normalizarBidireccional(ch1_us);
intavanceCmd =normalizarBidireccional(ch2_us);
intthrottleCmd =normalizarMagnitud(ch3_us);

controlarRobot(throttleCmd,avanceCmd,dirCmd);

if (millis()-ultimoPrint>=PRINT_INTERVAL_MS) {
ultimoPrint =millis();
imprimirEstado(ch1_us,ch2_us,ch3_us,dirCmd,avanceCmd,throttleCmd);
  }
}