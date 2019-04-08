//-----------------------------------------------------------------------------------------------------
// IFG - Campus Goiânia
// Curso: 
// Disciplina: 
// Professor: Carlos Roberto da Silveira Junior
// Objetivo: O Robô desvia de obstáculos a 30 cm de distância usando o sensor de ultrassom
// 
//-------------------------------------------------------------------------------------------------------

#define DIREITA  1
#define ESQUERDA 0

int lado = DIREITA;

// Definição dos pinos 

  int pinL1D = 2;
  int pinL1A = 19;
  int pinL2D = 3;
  int pinL2A = 18;
  int pinUS = 5;
  int pinEB = 6;
  int pinI4 = 7;
  int pinI3 = 8;
  int pinEA = 11;
  int pinI2 = 12;
  int pinI1 = 13;
  int pinTDIR = 14;
  int pinTESQ = 15;
  int pinAPITO = 17;

// Constante de velocidade divisor (1 - 10)
  int constVel = 1;
  
// Definicao de variaveis
  int sensLuz1 = 0;
  int sensLuz2 = 0;
  int valLuz1  = 0;
  int distancia =0;
  int velocidade =0;
  
  
  
void setup()
{
  pinMode(pinL1D, OUTPUT);
  pinMode(pinL2D, OUTPUT);
  pinMode(pinEA , OUTPUT);
  pinMode(pinEB , OUTPUT);
  pinMode(pinI1 , OUTPUT);
  pinMode(pinI2 , OUTPUT);
  pinMode(pinI3 , OUTPUT);
  pinMode(pinI4 , OUTPUT);
  pinMode(pinTDIR,  OUTPUT);
  pinMode(pinTESQ,  OUTPUT);
  pinMode(pinAPITO, OUTPUT);
  while(!digitalRead(pinTDIR));
  ligaApito();
  delay(1000);
  desligaApito();
  delay(1000);
  Serial.begin(9600);
  Serial.println("Iniciando processo");
}

void loop()
{
  if(digitalRead(pinTDIR))
  {
    ligaApito();
    parar();
    delay(100);
    andarAtras(); 
    delay(500);    
    virarEsquerda();
    delay(500);
    desligaApito();    
    andarFrente();
  }
  if(digitalRead(pinTESQ))
  {
    ligaApito();
    parar();
    delay(100);
    andarAtras(); 
    delay(500);    
    virarDireita();
    delay(500);
    desligaApito();    
    andarFrente();
  }
  distancia = ultrasom(); 
  if(distancia < 30){
    parar();
    delay(100);
    andarAtras(); 
    delay(500);
    if(lado == DIREITA){
      virarDireita();
      lado == ESQUERDA;
    }
    else{
      virarEsquerda();
      lado == DIREITA;    
    }     
    delay(500);
    desligaApito(); 
    andarFrente();
  }

}

void ligaApito()
{
  digitalWrite(pinAPITO, HIGH);
}

void desligaApito()
{
  digitalWrite(pinAPITO, LOW);
}

void teste_sensorLuz()
{
    while(1){
      sensorLuz(1);
      Serial.print("SensLuz1: ");
      Serial.print(sensLuz1);
      sensorLuz(2);
      Serial.print("  SensLuz2: ");
      Serial.println(sensLuz2);
      delay(1000);
  }
}

unsigned long ultrasom()
{
  
  long duration;
  // Rotina para sensor de ultrasom de 3 pinos
  pinMode(pinUS, OUTPUT);
  digitalWrite(pinUS, LOW);
  delayMicroseconds(2);
  digitalWrite(pinUS, HIGH);
  delayMicroseconds(5);
  digitalWrite(pinUS, LOW);
  pinMode(pinUS, INPUT);
  duration = pulseIn(pinUS, HIGH);
  return duration/29/2;


}
void sensorLuz(int sensLuz)
{
  int i;
  int tempSensLuz = 0;
  if(sensLuz == 1)
    digitalWrite(pinL1D, HIGH);
  if(sensLuz == 2)
    digitalWrite(pinL2D, HIGH);
  for(i = 0; i < 20; i++)
  {
    if(sensLuz == 1)
      tempSensLuz += analogRead(pinL1A);
    if(sensLuz == 2)
      tempSensLuz += analogRead(pinL2A);
    delay(10);
  }
  if(sensLuz == 1){
    digitalWrite(pinL1D, LOW);
    sensLuz1 = tempSensLuz/i; 
  }
  if(sensLuz == 2){
    digitalWrite(pinL2D, LOW);
    sensLuz2 = tempSensLuz/i;
  }
}


void andarFrenteVel(int vel) 
{
  digitalWrite(pinI1,HIGH);
  digitalWrite(pinI2,LOW);
  digitalWrite(pinI3,LOW);
  digitalWrite(pinI4,HIGH);
  analogWrite(pinEA,vel/constVel);
  analogWrite(pinEB,int(vel*1.15)/constVel);
}

void andarFrente() 
{
  digitalWrite(pinI1,HIGH);
  digitalWrite(pinI2,LOW);
  digitalWrite(pinI3,LOW);
  digitalWrite(pinI4,HIGH);
  analogWrite(pinEA,160/constVel);
  analogWrite(pinEB,180/constVel);
}

void virarDireita() 
{
  digitalWrite(pinI1,LOW);
  digitalWrite(pinI2,LOW);
  digitalWrite(pinI3,HIGH);
  digitalWrite(pinI4,LOW);
  analogWrite(pinEA,200/constVel);
  analogWrite(pinEB,200/constVel);
}

void virarEsquerda() 
{
  digitalWrite(pinI1,LOW);
  digitalWrite(pinI2,HIGH);
  digitalWrite(pinI3,LOW);
  digitalWrite(pinI4,LOW);
  analogWrite(pinEA,200/constVel);
  analogWrite(pinEB,200/constVel);
}

void andarAtras() 
{
  digitalWrite(pinI1,LOW);
  digitalWrite(pinI2,HIGH);
  digitalWrite(pinI3,HIGH);
  digitalWrite(pinI4,LOW);
  analogWrite(pinEA,200/constVel);
  analogWrite(pinEB,200/constVel);
}

void girar() 
{
  
  digitalWrite(pinI1,LOW);
  digitalWrite(pinI2,HIGH);
  digitalWrite(pinI3,LOW);
  digitalWrite(pinI4,HIGH);
  analogWrite(pinEA,200/constVel);
  analogWrite(pinEB,200/constVel);
  delay(190);
}

void parar() 
{
  digitalWrite(pinI1,HIGH);
  digitalWrite(pinI2,HIGH);
  digitalWrite(pinI3,HIGH);
  digitalWrite(pinI4,HIGH);
}
