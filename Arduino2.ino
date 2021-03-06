#include <Keypad.h>
#include <string.h>
int pirPin = 10;
int statePir = LOW;
int valPir = 0;

const byte Linhas = 4; 
const byte Colunas = 4; 
static volatile unsigned long time, elapsed, prev_time;
static volatile unsigned long elapsedPir, prev_timePir;
static volatile unsigned long elapsedPirBloc, prev_timePirBloc;

char pass [6]={"AB20"},lock [6]={"1425"};
int tentativas =3 ,estadoDaLock =0;//Estado a 0 siginica bloqueado
int acesoApagado = 1;
int disparado = 0 ;
char palavra[25]="";
char* letra = palavra;
int countL=0;//Count letras

char hexaKeys[Linhas][Colunas] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[Linhas] = {9, 8, 7, 6}; 
byte colPins[Colunas] = {5, 4, 3, 2}; 

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, Linhas, Colunas); 

void setup(){
  pinMode(10, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
 // pinMode(10, INPUT); // Sensor pir como input
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(A5, OUTPUT);
  Serial.begin(9600);
}

void loop(){
  time = millis();
  elapsedPir = time - prev_timePir;
  elapsedPirBloc = time - prev_timePirBloc;
  valPir = digitalRead(pirPin);  //Le o valor do Pir
  
  char customKey = customKeypad.getKey();
  elapsed = time - prev_time;
  if(estadoDaLock == 0 && valPir == HIGH){
     Serial.print("Bloqueado");
        digitalWrite(11, HIGH);
        digitalWrite(12, HIGH);
        disparado = 1 ;
  }
  if(elapsed >= 2000 && estadoDaLock == 0 && disparado == 0){
    prev_time = time;
    if(acesoApagado == 0){
        digitalWrite(12, LOW);
      acesoApagado = 1;
    }else{
        acesoApagado = 0;
      digitalWrite(12, HIGH);
    }      
  }
  
  if (customKey && customKey!= '#'){
    if(customKey == '*'){
      *letra = ' ';
    }else{
    *letra = customKey;
    }
    letra++;
    
  }else if(customKey== '#'){
    
    Serial.println(strcmp(palavra,pass));
    
    if((tentativas <= 0) && estadoDaLock == 0 && strcmp(palavra,pass)!=0 ){
      //time=millis();
       Serial.print("Bloqueado");
     // if(elapsed > 2){
        digitalWrite(11, HIGH);
        digitalWrite(12, HIGH);
      //}
      prev_time = time;
    }else if(strcmp(palavra,pass)!=0 && (tentativas != 0) && estadoDaLock==0){
       Serial.println("Tentativas--");
      tentativas --;
    }else if(strcmp(palavra,pass)==0 && (tentativas >= 0) && strcmp(palavra,lock)!=0){
       Serial.println("Desbloqueado");
        disparado = 0;
        tentativas = 3;
        digitalWrite(A5, HIGH); 
        digitalWrite(11, LOW);
        digitalWrite(12, LOW);
        PORTB = 0x00;
        estadoDaLock =1;
    }else if(strcmp(palavra,lock)== 0 && estadoDaLock == 1 ) {
      //if(time>){
      
      //}
      PORTC = 0X00;
      digitalWrite(A3, LOW);
      digitalWrite(A4, LOW);  
    digitalWrite(A5, LOW);  
      //digitalWrite(10, LOW);
      digitalWrite(11, LOW);  
    digitalWrite(12, LOW);
         Serial.println("Ficou Bloqueado");
      estadoDaLock =0;
    }
    Serial.print(palavra);
    memset( palavra, 0, sizeof(palavra) );//Limpa o array
    letra=palavra;
    Serial.println();
  }
    if (valPir == HIGH) {           // Verifica se o sensor está HIGH
 
      if (statePir == LOW && elapsedPir >= 100) {
         prev_timePir = time;
        Serial.println("Movimento detetado!"); 
        statePir = HIGH;       // Atualiza a variavel para HIGH
      }
  }else {
      if (statePir == HIGH && elapsedPir >= 200){
         prev_timePir = time;
         Serial.println("Movimento terminou!");
         statePir = LOW;       //  Atualiza a variavel para LOW
      }
  }
 
}
