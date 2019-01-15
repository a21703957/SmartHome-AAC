#include <dht.h>
#include <Wire.h>  
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#define T_AfastarPort 5000
Servo servoRua;
Servo servoPortao;
dht DHT;
int microfone = 2; //
int ledPin = 7; //Led no pino 7
int DHT11_PIN =8;\
int motorPin = 3;
int servoPor = 9;
int trigPin = 5;    // Trigger
int echoPin = 6;    // Echo
//int servoR = 10;
long duration, cm, inches;
int ldrPin = A1; //LDR no pino analígico A0
int sensAgua = A0; // Sensor de água analígico A1
int smokeA3 = A2;

int ldrValor = 0; //Valor lido do LDR
int recebidoA = 0;
LiquidCrystal_I2C lcd(0x27, 16, 2);

//_________
static volatile unsigned long time, elapsedLum, prev_timeLum;
static volatile unsigned long elapsedHum, prev_timeHum;
static volatile unsigned long elapsedHumMed, prev_timeHumMed;
static volatile unsigned long elapsedGas, prev_timeGas;
static volatile unsigned long elapsedPir, prev_timePir;
static volatile unsigned long elapsedPort, prev_timePort;
static volatile unsigned long elapsedPortD, prev_timePortD;
static volatile unsigned long elapsedGasTerm, prev_timeGasTerm;
static volatile unsigned long elapsedEscrever, prev_timeEscrever;

static volatile unsigned long elapsedLumTerm, prev_timeLumTerm;

int gasAtivo=0;
 
// Tempo máximo entre o pulso seguinte
unsigned long tempMaxSom = 1000; 
unsigned long tempMinSom = 300;  
unsigned long compriSonoro = 100; // Comprimento do som esperado
unsigned long startTime = 0;
//Som
boolean status_lights = false;
int palma = 0;
long inicio_alcance_detecao = 0;
long alcanceDetetado = 0;

int sensorThres = 420;// Sensor de fumo a partir da qual fica ativo
void setup() {
  servoRua.attach(10);
  servoPortao.attach(9);
  // initialize the LCD
  lcd.begin();
  
  // Turn on the blacklight
  lcd.setBacklight((uint8_t)1);

  // First row
  lcd.print("BEM VINDO!");

  // Second row
  lcd.setCursor(0,1);
  lcd.print("SMART HOME");
  pinMode(microfone, INPUT);//define a porta 2 como saída
  pinMode(ledPin,OUTPUT); //define a porta 7 como saída
  pinMode(motorPin, OUTPUT);//define a porta 3 como saída
  pinMode(servoPor, OUTPUT);//define a porta 9 como saída
  //pinMode(servoR, OUTPUT);//define a porta 10 como saída
  pinMode(ldrPin, INPUT);
  pinMode(sensAgua, INPUT);
  pinMode(smokeA3, INPUT);
   pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600); //Inicia a comunicação serial
}
void loop() {
  

  time = millis();
  elapsedLum = time - prev_timeLum;
  elapsedHum = time - prev_timeHum;
  elapsedGas = time - prev_timeGas;
  elapsedPir = time - prev_timePir;
  elapsedPort = time - prev_timePort;
  elapsedPortD = time - prev_timePortD;
  elapsedEscrever = time - prev_timeEscrever;
  elapsedHumMed = time - prev_timeHumMed;
  elapsedGasTerm = time - prev_timeGasTerm;
  elapsedLumTerm = time - prev_timeLumTerm;
  ldrValor = analogRead(ldrPin); //O valor lido será entre 0 e 1023 (luminusidade)
  //ALTERAR_______
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  cm = (duration/2) / 29.1;
  //som-----
  //Luminosidade
    ldrValor = analogRead(ldrPin); //O valor lido será entre 0 e 1023 (luminusidade)

  if (ldrValor >= 950 && elapsedLum >= 100){
    prev_timeLumTerm = time ;
    prev_timeLum = time;
    digitalWrite(ledPin,HIGH);
  }else if (ldrValor < 950 && elapsedLumTerm >= 200 && !status_lights){ // Caso os leds liguem e desliguem aumentar 
    prev_timeLum = time;
    digitalWrite(ledPin,LOW); // senão, apaga o led
  }
  
  int status_sensor = digitalRead(microfone);
  if (status_sensor == 0){
    if (palma == 0){
      inicio_alcance_detecao = alcanceDetetado = millis();
      palma++;
      Serial.print(palma);
    }else if (palma > 0 && millis()-alcanceDetetado >= 50){
      alcanceDetetado = millis();
      palma++;
      Serial.print(palma);
    }
  }
  if (millis()-inicio_alcance_detecao >= 400){
    if (palma == 2){
      if (!status_lights){
          status_lights = true;
          digitalWrite(ledPin, HIGH);
        }else if (status_lights){
          status_lights = false;
          digitalWrite(ledPin, LOW);
        }
    }
    palma = 0;
  }
  //Humidade e Temperatura
  if(elapsedHum >=10000){
    prev_timeHum = time;
    int chk = DHT.read11(DHT11_PIN);//(Temperatura e Humidade)
    // First row
   if(gasAtivo == 0 ){
      lcd.clear();
        lcd.setCursor(5,0);
      //lcd.print("TEMPERATURA= ");
      lcd.print(DHT.temperature);
      lcd.print(" C");
    
      // Second row
      lcd.setCursor(0,1);
      lcd.print("HUMIDADE= ");
      lcd.print(DHT.humidity);
      lcd.print("%");
    }


  }


//Sensor de gas
//Serial.println(analogRead(smokeA3));
  if(elapsedGas >= 100){
    //Serial.println("Entrei");
    prev_timeGas = time;
    int analogSensor = analogRead(smokeA3);
        
    if(analogSensor >= sensorThres ){//Verifica se o valor atual é superior a 400
         prev_timeGasTerm = time;    
      gasAtivo =1;
      digitalWrite(motorPin,HIGH);//Arranca a ventoinha
//      tone(buzzer, 1000, 200);
      //digitalWrite(ledPin, HIGH);
  }else if(elapsedGasTerm >= 2000){
      digitalWrite(motorPin, LOW);//Desliga-se a ventoinha
      gasAtivo = 0;
     // noTone(buzzer);
    }
    if(gasAtivo == 1){
      lcd.clear();
      lcd.setCursor(5,0);
      //lcd.print("TEMPERATURA= ");
     
      lcd.print("PERIGO");
    
      // Second row
      lcd.setCursor(6,1);
      lcd.print("GAS");
    }
      
  }
//Estendal
  if(digitalRead(sensAgua) == HIGH){
    servoRua.write(180);
  }else{
    servoRua.write(0);
  }
  
  //Garagem 
  if(elapsedPortD >= 250){
    prev_timePortD = time;
    if(cm > 15 && elapsedPort >= T_AfastarPort){
   
      servoPortao.write(80);
     
    }else if(cm <= 15 ){
      servoPortao.write(0);
       prev_timePort = time;
    }
  }
/*
  if(elapsedEscrever >= 10){
    prev_timeEscrever = time;
    Serial.println();
    Serial.print("|Gas ");
    Serial.print(analogRead(smokeA3));
    Serial.print(" |");
    Serial.print(cm);
    //Serial.print("cm | Mic ");
   // Serial.print(status_sensor);
    Serial.print(" | Temp ");
    Serial.print(DHT.temperature);
    Serial.print(" | Humi ");
    Serial.print(DHT.humidity);
    Serial.print(" | Lum  ");
    Serial.print(ldrValor);
    Serial.println();
  }
*/
Serial.print(palma);
}
