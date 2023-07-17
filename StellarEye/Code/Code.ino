/*
Microcontroller - ESP Wroom 32

ESP Wroom 32
Vin = Battery +ve
Gnd = Battery -ve

BMP280
VCC = 3V3
GND = GND
SCL = GPIO22
SDA = GPIO21

MPU6050
VCC = 3V3
GND = GND
SCL = GPIO22
SDA = GPIO21

NEO GPS
VCC = 3V3
RX = GPIO17
TX = GPIO16
GND = GND

Xbee 
VCC = 3V3 (sheild)
RX = GPIO04
TX = GPIO02
Gnd = Gnd

SD Card Reader
CS = GPIO5
SCK = GPIO18
MOSI = GPIO23
MISO = GPIO19
VCC = 3V3
Gnd = Gnd

LED
Anode = GPIO15
Cathode = Gnd

Buzzer
Anode = GPIO13
Cathode = Gnd

TIP122 npn(Drouge)
Base = GPIO12
Collector = battery +ve
Emitter = Gnd

TIP122 npn(Main)
Base = GPIO14
Collector = battery +ve
Emitter = Gnd

*/

#include <SD.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
File myFile;
TaskHandle_t task_loop1;

int buzzer=13,led=15,drogue_pin=12,main_pin=14;
const int CS=5,n=15;
String dataFile="/data.txt",logFile="/log.txt";
float pressure,curr_altitude=0,prev_altitude,temperature,acc_x,acc_y,acc_z,rot_x,rot_y,rot_z,latitude,longitude,avg=0,apogee,drogue_deployed,main_deployed,curr,prev,main_altitude=1500;
bool flag_drogue=false,flag_main=false;

void esploop1(void* pvParameters){
  setup1();

  for(;;){
    loop1();
  }
}

void setup() {
  delay(5000);
  xTaskCreatePinnedToCore(esploop1,"loop1",10000,NULL,1,&task_loop1,!ARDUINO_RUNNING_CORE);

  pinMode(buzzer,OUTPUT);
  pinMode(led,OUTPUT);

  digitalWrite(buzzer,LOW);
  digitalWrite(led,LOW);

  Serial.begin(9600);
  Serial2.begin(9600,SERIAL_8N1,16,17);

  if(SD.begin(CS)){
    Serial.println("SD initialized successfully! \n");  
    
    digitalWrite(buzzer,HIGH);
    digitalWrite(led,HIGH);
    delay(1000);
    digitalWrite(led,LOW);
    digitalWrite(buzzer,LOW);
  }
  else{
    Serial.println("Error initializing SD! \n");
    while(1){

    }
  }  

  if(SD.exists(dataFile)){
    Serial.println("Deleting the data file! \n");
    SD.remove(dataFile);
  }

  delay(1000);

  if(SD.exists(logFile)){
    Serial.println("Deleting the log file! \n");
    SD.remove(logFile);
  }

  delay(1000);
  
  myFile=SD.open(logFile,FILE_APPEND);

  if(myFile){
    Serial.println("Pressure(Pa),Altitude(ft),Temperature(C),Acceleration_x(m/s2),Acceleration_y(m/s2),Acceleration_z(m/s2),Rotation_x(deg),Rotation_y(deg),Rotation_z(deg),Latitude(deg),Longitude(deg)");
    Serial.println();
    myFile.println("Pressure(Pa),Altitude(ft),Temperature(C),Acceleration_x(m/s2),Acceleration_y(m/s2),Acceleration_z(m/s2),Rotation_x(deg),Rotation_y(deg),Rotation_z(deg),Latitude(deg),Longitude(deg)");
    myFile.println();

    if(bmp.begin(0x76)){
      Serial.println("BMP initialised successfully!");
      Serial.println();
      myFile.println("BMP initialised successfully!");
      myFile.println();
  
      digitalWrite(buzzer,HIGH);
      digitalWrite(led,HIGH);  
      delay(1000);
      digitalWrite(led,LOW);
      digitalWrite(buzzer,LOW);
    }
    else{
      Serial.println("Error initialising BMP!");
      myFile.println("Error initialising BMP!");
      while(1){

      }
    }

    delay(1000);

    if(mpu.begin(0x68)){
      Serial.println("MPU initialised successfully!");
      Serial.println();
      myFile.println("MPU initialised successfully!");
      myFile.println();
    
      digitalWrite(buzzer,HIGH);
      digitalWrite(led,HIGH);
      delay(1000);
      digitalWrite(led,LOW);
      digitalWrite(buzzer,LOW);
    }
    else{
      Serial.println("Error initialising MPU!");
      myFile.println("Error initialising MPU!");
      while(1){

      }
    }
    myFile.close();
  }
  else{
    Serial.println("Error opening file in setup!");
    while(1){

    }
  }
}

void setup1(){
  pinMode(drogue_pin,OUTPUT);
  pinMode(main_pin,OUTPUT);

  digitalWrite(drogue_pin,LOW);
  digitalWrite(main_pin,LOW);
}

void loop() {
  myFile=SD.open(dataFile,FILE_APPEND);

  if(myFile){
    getBmpData();
    getMpuData();
    getGpsData();
    myFile.close();
  }
  else{
    Serial.println("Error opening file in loop!");
    while(1){

    }
  }
  delay(50);
}


void loop1(){
  if(!flag_drogue){
    apogee_func();
  }
  else if(flag_drogue && !flag_main){
    if(curr_altitude<=main_altitude){
      main_func();
    }
  }
}

void apogee_func(){
  if(curr_altitude<prev_altitude){
    apogee=prev_altitude;
    curr=curr_altitude;
    prev=prev_altitude;
    for(int i=0;i<n;i++){
      avg=avg+(curr-prev);
      prev=curr;
      delay(200);
      curr=curr_altitude;
    }
    avg=avg/n;
    if(int(avg)<0){
      drogue_func();
    }
    else{
      avg=0;
    }
  }
}

void drogue_func(){
  digitalWrite(buzzer,HIGH);
  digitalWrite(led,HIGH);
  digitalWrite(drogue_pin,HIGH);
  drogue_deployed=curr_altitude;
  delay(10000);
  digitalWrite(drogue_pin,LOW);
  digitalWrite(led,LOW);
  digitalWrite(buzzer,LOW);
  flag_drogue=true;
}

void main_func(){  
  digitalWrite(buzzer,HIGH);
  digitalWrite(led,HIGH);
  digitalWrite(main_pin,HIGH);
  main_deployed=curr_altitude;
  delay(10000);
  digitalWrite(main_pin,LOW);
  digitalWrite(led,LOW);
  digitalWrite(buzzer,LOW);
  flag_main=true;
}

void getBmpData(){
  prev_altitude=curr_altitude;

  pressure=bmp.readPressure();
  curr_altitude=bmp.readAltitude()*3.28084;
  temperature=bmp.readTemperature();

  Serial.print(pressure);
  Serial.print(",");
  
  Serial.print(curr_altitude);
  Serial.print(",");

  Serial.print(temperature);
  Serial.print(",");

  myFile.print(pressure);
  myFile.print(",");
  
  myFile.print(curr_altitude);
  myFile.print(",");

  myFile.print(temperature);
  myFile.print(",");
}

void getMpuData(){

  sensors_event_t a,g,temp;
  mpu.getEvent(&a,&g,&temp);
  
  acc_x=a.acceleration.x;
  acc_y=a.acceleration.y;
  acc_z=a.acceleration.z;
  
  rot_x=g.gyro.x;
  rot_y=g.gyro.y;
  rot_z=g.gyro.z;

  Serial.print(acc_x);
  Serial.print(",");
    
  Serial.print(acc_y);
  Serial.print(",");
  
  Serial.print(acc_z);
  Serial.print(",");
  
  Serial.print(rot_x);
  Serial.print(",");
  
  Serial.print(rot_y);
  Serial.print(",");
  
  Serial.print(rot_z);
  Serial.print(",");

  myFile.print(acc_x);
  myFile.print(",");
    
  myFile.print(acc_y);
  myFile.print(",");
  
  myFile.print(acc_z);
  myFile.print(",");
  
  myFile.print(rot_x);
  myFile.print(",");
  
  myFile.print(rot_y);
  myFile.print(",");
  
  myFile.print(rot_z);
  myFile.print(",");

}

void getGpsData(){
  
  while(Serial2.available()>0){
    if(gps.encode(Serial2.read())){
      if(gps.location.isValid()){
        latitude=gps.location.lat();
        longitude=gps.location.lng();
      }
    }
  } 

  Serial.print(latitude,6);
  Serial.print(",");

  Serial.println(longitude,6);

  myFile.print(latitude,6);
  myFile.print(",");

  myFile.println(longitude,6);
}