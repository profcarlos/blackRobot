#include <Wire.h>
#include "RichShieldNTC.h"
#include "RichShieldLightSensor.h"
#include "RichShieldVoltageSensor.h"
#include "RichShieldTM1637.h"
#include <SimpleModbusSlave.h>

// https://www.embarcados.com.br/integracao-arduino-e-elipse-scada/
/* 
   SimpleModbusSlaveV10 supports function 3, 6 & 16.
   
   This example code will receive the adc ch0 value from the arduino master. 
   It will then use this value to adjust the brightness of the led on pin 9.
   The value received from the master will be stored in address 1 in its own
   address space namely holdingRegs[].
   
   In addition to this the slaves own adc ch0 value will be stored in 
   address 0 in its own address space holdingRegs[] for the master to
   be read. The master will use this value to alter the brightness of its
   own led connected to pin 9.
   
   The modbus_update() method updates the holdingRegs register array and checks
   communication.

   Note:  
   The Arduino serial ring buffer is 64 bytes or 32 registers.
   Most of the time you will connect the arduino to a master via serial
   using a MAX485 or similar.
 
   In a function 3 request the master will attempt to read from your
   slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
   and two BYTES CRC the master can only request 58 bytes or 29 registers.
 
   In a function 16 request the master will attempt to write to your 
   slave and since a 9 bytes is already used for ID, FUNCTION, ADDRESS, 
   NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
   54 bytes or 27 registers.
 
   Using a USB to Serial converter the maximum bytes you can send is 
   limited to its internal buffer which differs between manufactures. 
*/

#define NTC_PIN A1 //SIG pin of NTC module connect to A1 of IO Shield, that is pin A1 of OPEN-SMART UNO R3
NTC temper(NTC_PIN);

#define LIGHTSENSOR_PIN A2//SIG pin of Rocker Switch module connect to A0 of IO Shield, that is pin A2 of OPEN-SMART UNO R3
LightSensor lightsensor(LIGHTSENSOR_PIN);  

#define VOL_SENSOR A0//The SIG pin connects A3 pin of OPEN-SMART UNO R3
VoltageSensor voltage(VOL_SENSOR);

#define CLK 10//CLK of the TM1637 IC connect to D10 of OPEN-SMART UNO R3
#define DIO 11//DIO of the TM1637 IC connect to D11 of OPEN-SMART UNO R3
TM1637 disp(CLK,DIO);

#define LED_VERM 4
#define LED_AMAR 7
#define LED_VERD 5
#define LED_AZUL 6

// Using the enum instruction allows for an easy method for adding and 
// removing registers. Doing it this way saves you #defining the size 
// of your slaves register array each time you want to add more registers
// and at a glimpse informs you of your slaves register layout.

//////////////// registers of your slave ///////////////////
enum 
{     
  // just add or remove registers and your good to go...
  // The first register starts at address 0
  TEMP_VAL, 
  LUM_VAL,    
  POT_VAL,        
  HOLDING_REGS_SIZE // leave this one
  // total number of registers for function 3 and 16 share the same register array
  // i.e. the same address space
};

unsigned int holdingRegs[HOLDING_REGS_SIZE]; // function 3 and 16 register array
////////////////////////////////////////////////////////////

void setup()
{
  /* parameters(HardwareSerial* SerialPort,
                long baudrate, 
		unsigned char byteFormat,
                unsigned char ID, 
                unsigned char transmit enable pin, 
                unsigned int holding registers size,
                unsigned int* holding register array)
  */
  
  /* Valid modbus byte formats are:
     SERIAL_8N2: 1 start bit, 8 data bits, 2 stop bits
     SERIAL_8E1: 1 start bit, 8 data bits, 1 Even parity bit, 1 stop bit
     SERIAL_8O1: 1 start bit, 8 data bits, 1 Odd parity bit, 1 stop bit
     
     You can obviously use SERIAL_8N1 but this does not adhere to the
     Modbus specifications. That said, I have tested the SERIAL_8N1 option 
     on various commercial masters and slaves that were suppose to adhere
     to this specification and was always able to communicate... Go figure.
     
     These byte formats are already defined in the Arduino global name space. 
  */
	
  modbus_configure(&Serial, 9600, SERIAL_8N2, 1, 2, HOLDING_REGS_SIZE, holdingRegs);

  // modbus_update_comms(baud, byteFormat, id) is not needed but allows for easy update of the
  // port variables and slave id dynamically in any function.
  modbus_update_comms(9600, SERIAL_8N2, 1);
  disp.init();//The initialization of the display
  pinMode(LED_VERM, OUTPUT);
  pinMode(LED_VERD, OUTPUT);
  pinMode(LED_AMAR, OUTPUT);
  pinMode(LED_AZUL, OUTPUT);
}

void loop()
{
  float temperatura;
  float luminosidade;
  float potenciometro;
  // modbus_update() is the only method used in loop(). It returns the total error
  // count since the slave started. You don't have to use it but it's useful
  // for fault finding by the modbus master.
  
  temperatura = temper.getTemperature();//get temperature
  float Rsensor = lightsensor.getRes();
  luminosidade = 325*pow(Rsensor,-1.4);
  potenciometro = voltage.read();
  
  disp.display(temperatura);
  digitalWrite(LED_AMAR, HIGH);
  delay(200);
  digitalWrite(LED_AMAR, LOW);
  disp.display(luminosidade);
  digitalWrite(LED_AZUL, HIGH);
  delay(200);
  digitalWrite(LED_AZUL, LOW);
  disp.display(potenciometro);
  digitalWrite(LED_VERD, HIGH);
  delay(200);
  digitalWrite(LED_VERD, LOW);

  digitalWrite(LED_VERM, HIGH);
  holdingRegs[TEMP_VAL] = int(temperatura);
  holdingRegs[LUM_VAL]  = int(luminosidade);
  holdingRegs[POT_VAL]  = int(potenciometro);
  modbus_update();
  delay(200);
  digitalWrite(LED_VERM, LOW);
  
  /* Note:
     The use of the enum instruction is not needed. You could set a maximum allowable
     size for holdinRegs[] by defining HOLDING_REGS_SIZE using a constant and then access 
     holdingRegs[] by "Index" addressing. 
     I.e.
     holdingRegs[0] = analogRead(A0);
     analogWrite(LED, holdingRegs[1]/4);
  */
  
}
