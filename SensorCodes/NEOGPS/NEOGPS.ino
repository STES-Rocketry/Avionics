#include <TinyGPSPlus.h>

TinyGPSPlus gps;

float latitude,longitude;

void setup() {
  Serial.begin(9600);    
}

void loop() {
  
  getGpsData();
  delay(1000);

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