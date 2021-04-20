#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

// LSM9DS1 code
#include <Wire.h>   //for i2c comms
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  //not used in this demo but required!

//i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();  //???

//define SPI i/o
#define LSM9DS1_SCK A5    //clock signal (can double as SCL for i2c)
#define LSM9DS1_MISO 12   //miso signal
#define LSM9DS1_MOSI A4   //mosi signal (can double as SDA for i2c)
#define LSM9DS1_XGCS 6    //??? <<- can these be commented out since they get overwritten?
#define LSM9DS1_MCG 5     //??? <<- can these be commented out since they get overwritten?

//initialize moving average arrays
int count = 10; //used for shifting moving avg values within arrays -- MUST be same size as arrays
float averagePitchArray[] = {0,0,0,0,0,0,0,0,0,0};  //array for pitch axis data
float averageRollArray[] = {0,0,0,0,0,0,0,0,0,0};   //array for roll axis data
float averageYawArray[] = {0,0,0,0,0,0,0,0,0,0};    //array for yaw axis data
float averageStressArray[] = {0,0,0,0,0,0,0,0,0,0}; //array for flex sensor data

int countdown;  //???

//???
union float_byte
{
  float number;
  uint8_t bytes[4];
};

float_byte averagePitch;  //declares TX byte for pitch axis data???
float_byte averageRoll;   //declares TX byte for roll axis data???
float_byte averageYaw;    //declares TX byte for yaw axis data???
float_byte averageStress; //declares TX byte for stress sensor data???

float lastYaw = 90.0; //used to integrate yaw signal
float gyroDrift = 0.006;  //counteracts gyro drift when integrating
                          //^To determine value, observe gyro signal on serial plotter/monitor
                          //...with no movement, and see how much gyro signal drifts per sample.
                          //...Then, subtract this number from total via choosing 'gyroDrift' value.

float_byte currentRot;    //???

//set up LSM9DS1
void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);  //can change 2G,4G,8G,16G
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);   //can change 4GAUSS,8GAUSS,12GAUSS,16GAUSS

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);  //can change 245DPS,500DPS,2000DPS
}

void setup()
{
  //14240 port

  Serial.begin(115200); //launch serial monitor/plotter at 115200 baud rate

  Mirf.cePin = 9;   //make pin D9 (Arduino Nano) the CE pin (nRF24L01 module)
  Mirf.csnPin = 10; //make pin D10 (Nano) the CSN pin (nRF24L01 module)
  Mirf.spi = &MirfHardwareSpi;  //???
  Mirf.init();      //initialize nRF24L01 module
  //Set the receiving identifier "Sen01"
  Mirf.setRADDR((byte *)"Sen01"); //??? //is this line redundant w/ similar line in transmit()? can that one replace this one?
  //Set the number of bytes sent and received at a time, here send an integer, write sizeof (unsigned int), actually equal to 2 bytes
  Mirf.payload = 16;  //sizeof(unsigned int);
  //Sending channel, can fill 0~128, send and receive must be consistent
  Mirf.channel = 3;
  Mirf.config();  //???
    //Note that one Arduino writes Sender.ino and the other writes Receiver.ino
    //The identifier here is written to Sender.ino
  //Serial.println("I'm Sender...");

  //LSM9DS1 code
  while(!Serial){
    delay(1);   //will pause Zero, Leonardo, Nano, etc. until serial console opens
  }

  //Serial.println("LSM9DS1 data read demo);

  //Try to initialize chip and warn if chip can't be detected
  if(!lsm.begin()){
    //Serial.println("Oops...Unable to initialize the LSM9DS1. Check your wiring!");
    while(1);
  }
  //Serial.println("Found LSM9DS1 9DOF");

  //Helper to set the defaunt scaling we want & finish setting up LSM9DS1 -- see above!
  setupSensor();

  currentRot.number = 0;  //???
  countdown = 100;        //???
}

void loop()
{
  //Flex sensor values: ~722 (min), ~822 (max)

  //???
  if(countdown > 0){
    countdown--;
  }

  //Read data from LSM9DS1
  lsm.read();
  sensors_event_t a,m,g,temp;   //set a new sensor event
  lsm.getEvent(&a,&m,&g,&temp); //get a new sensor event

  //Servo Rotation???: 0-90-180

  //Shift moving average array values to make room for new sample
  for(int i=0; i < count-1; i++){
    averagePitchArray[i] = averagePitchArray[i+1];
    averageRollArray[i] = averageRollArray[i+1];
    averageYawArray[i] = averageYawArray[i+1];
    averageStressArray[i] = averageStressArray[i+1];
  }
  
  //Save new sample value to [newly free spot in] array
  averagePitchArray[count-1] = a.acceleration.x;  //CAN USE GYRO DATA INSTEAD IF DESIRED
  averageRollArray[count-1] = a.acceleration.y;   //CAN USE GYRO DATA INSTEAD IF DESIRED
  averageYawArray[count-1] = g.gyro.z;            //CAN USE ACCEL DATA INSTEAD IF DESIRED
  averageStressArray[count-1] = analogRead(A0);
  
  //???
  averagePitch.number = 0;
  averageRoll.number = 0;
  averageYaw.number = 0;
  averageStress.number = 0;
  
  //???
  for(int i=0; i < count; i++){
    averagePitch.number += averagePitchArray[i];
    averageRoll.number += averageRollArray[i];
    averageYaw.number += averageYawArray[i];
    averageStress.number += averageStressArray[i];
  }
  
  //???
  averagePitch.number /= count;
  averageRoll.number /= count;
  averageYaw.number /= count;
  averageStress.number /= count;
  
  //Determine servo movement for yaw axis (uses gyroscope)
  if((averageRoll.number > 0.05 || averageRoll.number < -0.05) && countdown == 0){
    currentRot.number += averageRoll.number;  //calculate currentRot
  }
  if(currentRot.number > 90){
    currentRot.number = 90;                   //normalize currentRot
  }
  else if(currentRot.number < -90){
    currentRot.number = -90;                  //normalize currentRot
  }
  /////////Serial.print(currentRot.number);
  /////////Serial.print("\t");
  /////////Serial.print(averageRoll.number);
  /*
   * Serial.print(averageYaw.number);
   * Serial.print("\t");
   * Serial.print(a.acceleration.x);
   * Serial.print("\t");
   * Serial.print(averagePitch.number);
   * Serial.print("\t");
   * Serial.print(a.acceleration.y);
   */

  //Determine servo movement for pitch & roll axes (uses accelerometer)
  averagePitch.number = 90 + (averagePitch.number*9.1);
  averageRoll.number = 90 + (averageRoll.number*9.1);

  //Determine engine thrust from flex sensor val
  averageStress.number = (averageStress.number - 845)*2.0;  //800 is min value for stress sensor
  if(averageStress.number < 20){
    averageStress.number = 20;                 //set baseline at 20 to prevent negative values (leads to overflow at RX end)
  }


  //Integrate yaw signal
  averageYaw.number = averageYaw.number + lastYaw - gyroDrift;  //add previous value to running sum
  lastYaw = averageYaw.number;                      //save new value for next loop
  
  //TRANSMIT DATA
  transmit(averagePitch,averageRoll,averageYaw,averageStress);
  
  Serial.print(averagePitch.number);
  Serial.print("\t");
  Serial.print(averageRoll.number);
  Serial.print("\t");
  Serial.print(averageYaw.number);
  Serial.print("\t");
  Serial.println(averageStress.number);
}

void transmit(float_byte pitch, float_byte roll ,float_byte yaw ,float_byte stress)
{
  roll.number += 90;  //???

  //Defing a scratchpad array with a size of Mirf.payload (4 bytes, yes???)
  byte data[Mirf.payload];  //???

  data[0] = pitch.bytes[0];
  data[1] = pitch.bytes[1];
  data[2] = pitch.bytes[2];
  data[3] = pitch.bytes[3];

  data[4] = roll.bytes[0];
  data[5] = roll.bytes[1];
  data[6] = roll.bytes[2];
  data[7] = roll.bytes[3];

  data[8] = yaw.bytes[0];
  data[9] = yaw.bytes[1];
  data[10] = yaw.bytes[2];
  data[11] = yaw.bytes[3];

  data[12] = stress.bytes[0];
  data[13] = stress.bytes[1];
  data[14] = stress.bytes[2];
  data[15] = stress.bytes[3];

  //Settings send data to "serv1"
  Mirf.setTADDR((byte *)"Rec01");   //specifies identification byte for TX (make RX expect this same byte)???
  Mirf.send(data);

  //The next step can only be performed after the 'while' loop function transmission is completed
  while(Mirf.isSending()){}
  //delay(10);
}
