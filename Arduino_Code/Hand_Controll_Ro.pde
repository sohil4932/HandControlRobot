/**
 * © Sohil Patel 2013, Some Rights Reserved
 *
 *	This work is licensed under the Creative Commons Attribution-ShareAlike 3.0 Unported License. 
 *  To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/.
 * 
 * 
 * */

#include <IRremote.h>
int mleft1 = 9;
int mleft2 = 18;
int mright1 = 17;
int mright2 = 16;

int RECV_PIN = 5;

IRrecv irrecv(RECV_PIN);
void drive(unsigned long data);
decode_results results;

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
}

void loop() {
  if (irrecv.decode(&results)) {
    drive(results.value);
    irrecv.resume(); // Receive the next value
  }
}

void drive(unsigned long data)
{
byte pwm = data & 0x0FF;
byte dir = (data & 0xF00)>>8;

if(pwm != 0)
{
switch(dir)
{
case 0:
        analogWrite(mright1,pwm);
        analogWrite(mright2,0);  
        analogWrite(mleft1,pwm);
        analogWrite(mleft2,0);
        break;
case 1:
        analogWrite(mright1,0);
        analogWrite(mright2,pwm);  
        analogWrite(mleft1,pwm);
        analogWrite(mleft2,0);
        break;
case 2:
        analogWrite(mright1,pwm);
        analogWrite(mright2,0);  
        analogWrite(mleft1,0);
        analogWrite(mleft2,pwm);
        break;
case 3;
        analogWrite(mright1,0);
        analogWrite(mright2,pwm);  
        analogWrite(mleft1,0);
        analogWrite(mleft2,pwm);
        break;
default:
        break;
}
}
else
{
        analogWrite(mright1,0);
        analogWrite(mright2,0);  
        analogWrite(mleft1,0);
        analogWrite(mleft2,0);
}
delay(200);
}
