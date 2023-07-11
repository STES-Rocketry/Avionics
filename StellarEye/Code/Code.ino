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
float pressure,curr_altitude=0,last_altitude,temperature,acc_x,acc_y,acc_z,rot_x,rot_y,rot_z,latitude,longitude,avg=0,apogee,apogee1,drogue_deployed,drogue_deployed1,main_deployed,main_deployed1,down,up,main_altitude=1500;
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

  if(SD.exists(logFile)){
    Serial.println("Deleting the log file! \n");
    SD.remove(logFile);
  }

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
    apogee=apogee_func();
  }
  else if(flag_drogue && !flag_main){
    if(curr_altitude<=main_altitude){
      main_deployed=main_func(main_altitude);
    }
  }
}

void getBmpData(){

  last_altitude=curr_altitude;

  pressure=bmp.readPressure();
  curr_altitude=bmp.readAltitude()*3.28084;
  temperature=bmp.readTemperature();

  Serial.print("Pressure : ");
  Serial.print(pressure);
  Serial.println(" Pa");
  
  Serial.print("Altitude : ");
  Serial.print(curr_altitude);
  Serial.println(" ft");

  Serial.print("Temperature : ");
  Serial.print(temperature);
  Serial.println(" C");

  Serial.println();

  myFile.print("Pressure : ");
  myFile.print(pressure);
  myFile.println(" Pa");
  
  myFile.print("Altitude : ");
  myFile.print(curr_altitude);
  myFile.println(" ft");

  myFile.print("Temperature : ");
  myFile.print(temperature);
  myFile.println(" C");

  myFile.println();
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

  Serial.print("Acceleration_x : ");
  Serial.print(acc_x);
  Serial.print(" m/s2");
    
  Serial.print("          Acceleration_y : ");
  Serial.print(acc_y);
  Serial.print(" m/s2");
  
  Serial.print("          Acceleration_z : ");
  Serial.print(acc_z);
  Serial.println(" m/s2");
  
  Serial.print("Rotation_x     : ");
  Serial.print(rot_x);
  Serial.print(" deg");
  
  Serial.print("          Rotation_y     : ");
  Serial.print(rot_y);
  Serial.print(" deg");
  
  Serial.print("           Rotation_z     : ");
  Serial.print(rot_z);
  Serial.println(" deg");

  Serial.println();

  myFile.print("Acceleration_x : ");
  myFile.print(acc_x);
  myFile.print(" m/s2");
    
  myFile.print("          Acceleration_y : ");
  myFile.print(acc_y);
  myFile.print(" m/s2");
  
  myFile.print("          Acceleration_z : ");
  myFile.print(acc_z);
  myFile.println(" m/s2");
  
  myFile.print("Rotation_x     : ");
  myFile.print(rot_x);
  myFile.print(" deg");
  
  myFile.print("          Rotation_y     : ");
  myFile.print(rot_y);
  myFile.print(" deg");
  
  myFile.print("           Rotation_z     : ");
  myFile.print(rot_z);
  myFile.println(" deg");

  myFile.println();
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

  Serial.print("Latitude : ");
  Serial.print(latitude,6);
  Serial.println(" deg");

  Serial.print("Longitude : ");
  Serial.print(longitude,6);
  Serial.println(" deg");

  Serial.println();

  myFile.print("Latitude : ");
  myFile.print(latitude,6);
  myFile.println(" deg");

  myFile.print("Longitude : ");
  myFile.print(longitude,6);
  myFile.println(" deg");

  myFile.println();
}

float apogee_func(){
  if(curr_altitude<last_altitude){
    apogee1=last_altitude;
    down=curr_altitude;
    up=last_altitude;
    for(int i=0;i<n;i++){
      avg=avg+(down-up);
      up=down;
      delay(200);
      down=curr_altitude;
    }
    avg=avg/n;
    if(int(avg)<0){
      drogue_deployed=drogue_func();
    }
    else{
      avg=0;
    }
    return apogee1;
  }
}

float drogue_func(){
  digitalWrite(buzzer,HIGH);
  digitalWrite(led,HIGH);
  digitalWrite(drogue_pin,HIGH);
  drogue_deployed1=curr_altitude;
  delay(10000);
  digitalWrite(drogue_pin,LOW);
  digitalWrite(led,LOW);
  digitalWrite(buzzer,LOW);
  flag_drogue=true;

  return drogue_deployed1;
}

float main_func(int main_altitude){  
  digitalWrite(buzzer,HIGH);
  digitalWrite(led,HIGH);
  digitalWrite(main_pin,HIGH);
  main_deployed1=curr_altitude;
  delay(10000);
  digitalWrite(main_pin,LOW);
  digitalWrite(led,LOW);
  digitalWrite(buzzer,LOW);
  flag_main=true;
  
  return main_deployed1;
}