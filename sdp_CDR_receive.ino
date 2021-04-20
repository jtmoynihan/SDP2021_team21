#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

#include <Servo.h>

//Servo definition
Servo servo_pitch;
Servo servo_roll;
Servo servo_roll2;
Servo servo_yaw;

int led = 3;  //led at pin D2 of Arduino Nano

//???
union float_byte
{
  float number;
  uint8_t bytes[4];
};

//Initialize moving average arrays
float_byte pitch;
float_byte roll;
float_byte yaw;
float_byte stress;

//Initialize storage/reference variables for detecting packet loss
float lastRoll = 180.0;
float lastYaw = 90.0;
float lastStress = 0.0;

void setup()
{
  //14230 port

  //Attach servos to output pins
  servo_pitch.attach(6);  //attaching pitch servo to digital pin_out(6)
  servo_roll.attach(7);   //attaching roll servo to digital pin_out(7)
  servo_roll2.attach(8);  //attaching roll2 servo to digital pin_out(8)
  servo_yaw.attach(5);    //attaching yaw servo to digital pin_out(5)

  pinMode(led,OUTPUT);     //D2 is output for led pin

  //1440 port
  Serial.begin(115200); //launch serial monitor at 115200 baud rate

  //---Initial part, CAN'T be modified at ANY time---
  Mirf.cePin = 9;   //make pin D9 (Arduino) the CE pin (nRF24L01)
  Mirf.csnPin = 10; //make pin D10 (Arduino) the CSN pin (nRF24L01)
  Mirf.spi = &MirfHardwareSpi;  //???
  Mirf.init();      //initialization of nRF24L01
  //---Configuration part, can be modified at any time---
  //Set the receiving identifier byte "Rec01"
  Mirf.setRADDR((byte *)"Rec01");
  //Set number of bytes sent and received at a time, here sent an integer
  Mirf.payload = 16;  //Write sizeof(unsigned int), which is actually equal to 2 bytes
  //Sending channel, can fill 0~128, send and receive must be consistent
  Mirf.channel = 3;
  Mirf.config();  //???

  servo_pitch.write(90);
  servo_roll.write(90);
  servo_roll2.write(90);
  servo_yaw.write(90);
}

void loop()
{
  //Define a scratchpad array with a size of Mirf.payload (4 bytes, yes???)
  byte data[Mirf.payload];
  /*
   * if(Mirf.dataReady()){  //waiting for the prepared/received data
   *   Mirf.getData(data);  //receive data to data[] array
   *   //data[1] <<- move left 8 bits and data[0] merge, reorganize data
   *   adata = (unsigned int)((data[1] << 8 | data[0]);
   */

  if(Mirf.dataReady()){   //waiting for the prepared/received data
    Mirf.getData(data);   //receive data and assign to data[] 'scratchpad' array
  }

  pitch.bytes[0] = data[0];
  pitch.bytes[1] = data[1];
  pitch.bytes[2] = data[2];
  pitch.bytes[3] = data[3];

  roll.bytes[0] = data[4];
  roll.bytes[1] = data[5];
  roll.bytes[2] = data[6];
  roll.bytes[3] = data[7];

  yaw.bytes[0] = data[8];
  yaw.bytes[1] = data[9];
  yaw.bytes[2] = data[10];
  yaw.bytes[3] = data[11];

  stress.bytes[0] = data[12];
  stress.bytes[1] = data[13];
  stress.bytes[2] = data[14];
  stress.bytes[3] = data[15];

  //////////////////////////////////
  //Serial.print(pitch.number);
  //Serial.print("\t");
  ////////////Serial.print(roll.number);
  ////////////Serial.print("\t");
  ////////////Serial.println(180-roll.number);
  //Serial.print(yaw.number);
  ////////////Serial.print("\t");
  //Serial.println(stress.number);
  //////////////////////////////////

  //Actuate servos/led from received data
  servo_pitch.write(pitch.number*1.5 - 40.0);
  if(roll.number >= 3.0){
    servo_roll.write(roll.number/2.0);
    servo_roll2.write(roll.number/2.0);
  }
  else{
    servo_roll.write(lastRoll/2.0);
    servo_roll2.write(lastRoll/2.0);
  }
  if(yaw.number >= 3.0){              //
    servo_yaw.write(yaw.number);      //
  }                                   //
  else{                               //
    servo_yaw.write(lastYaw);         //
  }                                   //
  //servo_yaw.write(yaw.number);

  stress.number = stress.number - 20.0;   //need to subtract offset used to prevent negative vals -> overflow
  if(stress.number < 0.0){            ///
    stress.number = lastStress;       ///
  }                                   ///
  else{
    
  }                                   ///
  analogWrite(led,stress.number);
  Serial.println(stress.number);
  
  //Update correction variables, but only if new value isn't incorrect itself
  if(roll.number >= 3.0){
    lastRoll = roll.number;
  }
  else{}
  if(yaw.number >= 3.0){              ///
    lastYaw = yaw.number;             ///
  }                                   ///
  else{}                              ///
  if(stress.number >= 0.0){           ///
    lastStress = stress.number;       ///
  }                                   ///
  else{}                              ///
}
