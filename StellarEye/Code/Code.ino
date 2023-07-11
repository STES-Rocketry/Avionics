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
RX = GPI17
TX = GPI16
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

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
TaskHandle_t task_loop1;

int buzzer=13,led=15,drogue_pin=12,main_pin=14;
const int n=15;
float pressure,curr_altitude=0,last_altitude,temperature,acc_x,acc_y,acc_z,rot_x,rot_y,rot_z,latitude,longitude,avg=0,apogee,apogee1,drogue_deployed,drogue_deployed1,main_deployed,main_deployed1,down,up,main_altitude=1500;
bool flag_drogue=false,flag_main=false;

void esploop1(void* pvParameters){
  setup1();
  for(;;){
    loop1();
  }
}

void setup() {
  xTaskCreatePinnedToCore(esploop1,"loop1",10000,NULL,1,&task_loop1,!ARDUINO_RUNNING_CORE);  

  pinMode(buzzer,OUTPUT);
  pinMode(led,OUTPUT);

  digitalWrite(buzzer,LOW);
  digitalWrite(led,LOW);

  Serial.begin(9600);

  delay(5000);

  if(bmp.begin(0x76)){
    digitalWrite(buzzer,HIGH);
    digitalWrite(led,HIGH);
    Serial.println("BMP initialised successfully!");
    Serial.println();
    delay(1000);
    digitalWrite(led,LOW);
    digitalWrite(buzzer,LOW);
  }
  else{
    Serial.println("Error initialising BMP!");
    while(1){

    }
  }

  delay(1000);

  if(mpu.begin(0x68)){
    digitalWrite(buzzer,HIGH);
    digitalWrite(led,HIGH);
    Serial.println("MPU initialised successfully!");
    Serial.println();
    delay(1000);
    digitalWrite(led,LOW);
    digitalWrite(buzzer,LOW);
  }
  else{
    Serial.println("Error initialising MPU!");
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
  getBmpData();
  getMpuData();
  getGpsData();
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