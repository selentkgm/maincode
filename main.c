#include <checksum.h>
#include <mavlink_conversions.h>
#include <mavlink_get_info.h>
#include <mavlink_helpers.h>
#include <mavlink_sha256.h>
#include <mavlink_types.h>
#include <protocol.h>
#include <SoftwareSerial.h>
#include<Servo.h>
#include<ezButton.h>
#include <ezBuzzer.h>
#define servoPin 9 //pin

ezButton limitSwitch //pin
int buzzer_pin= //pin
Servo servo;

ezBuzzer sesliIkaz(buzzer_pin);

unit8_t sistemTuru = MAV_TYPE_HEXAROTOR;
unit8_t otopilotTuru =  MAV_AUTOPILOT_PIXHAWK;
unit8_t sistemModu = MAV_MODE_AUTO_ARMED;
unit8_t sistemDurumu = MAV_STATE_STANDBY; 

uint8_t mav1[MAVLINK_MAX_PACKET_LEN];
uint8_t mav2[MAVLINK_MAX_PACKET_LEN]; 

int yukseklik;
int buton = ?;               
int butondurum = 0; 
int sistemID    =1;
int companionID = 1;
int otopilotID  =1;
int bayrak;

void setup(){
  Serial.begin(9600);
  pinMode(buton,INPUT);

  heartbeat.sistemModu =  MAV_MODE_AUTO_ARMED; 
  heartbeat.custom_mode = --;// ' mod yaz'
  heartbeat.sistemDurumu = MAV_STATE_STANDBY;
  limitSwitch.setDebounceTime(50); //50 ms ile kontrol ediliyor
  servo.attach(servoPin);
}
int sistemHazir(){
     
    mavlink_command_long_t komut;
    komut.target_system = 1;
    komut.target_component = 1;
    komut.command = ;
    komut.confirmation =true;
 
   mavlink_message_t mesaj1;
   mavlink_msg_command_long_encode(sistemID, companionID, &mesaj1, &komut);

  
   int gonder3= write_message(mesaj1);

     return gonder1;
}

int kalkısModu(){//takeoff
     
    mavlink_command_long_t komut;
    komut.target_system = 1;
    komut.target_component = 1;
    komut.command = MAV_CMD_NAV_TAKEOFF;
    komut.confirmation =true;
 
   mavlink_message_t mesaj2;
   mavlink_msg_command_long_encode(sistemID, companionID, &mesaj2, &komut);

  
   int gonder3= write_message(mesaj2);

     return gonder2;
}

int irtifaSabitle(){
   if(bayrak==1)
     continue;
     
    mavlink_command_long_t komut;
    komut.target_system = 1;
    komut.target_component = 1;
    komut.command =  MAV_CMD_NAV_LOITER_TIME;
    komut.confirmation =true;
 
   mavlink_message_t mesaj3;
   mavlink_msg_command_long_encode(sistemID, companionID, &mesaj3, &komut);

  
   int gonder= write_message(mesaj3);
   bayrak=2;
     
   else
     bayrak=1;
   
     return gonder3;
}
int landKomutu(){
 
    mavlink_command_long_t komut;
    komut.target_system    = 1;
    komut.target_component = 1;
    komut.command          = MAV_CMD_NAV_LAND ;
    komut.confirmation     =true;
 
   mavlink_message_t mesaj4;
   mavlink_msg_command_long_encode(sistemID, companionID, &mesaj4, &komut);
     
     int gonder2= write_message(mesaj4)
     
     return gonder4;
}

void loop()
{
  buzzer.loop();
  button.loop();
  
  if(yukseklik==410)
    degisken=1;
  else if(yukseklik==700)
    //1.ayrılma gerçekleşiyor
    
  else if(yukseklik==410 && degisken==1){
    bayrak=1;
     
    servo.write(90);   //2.ayrılma gerçekleşti
   }
  delay(1500); //görev yükünün ayrılması için beklenir

  //guided modu 
  kalkisModu(); //takeoff
  //hız değişkenleri yapılacak
  
  else if(yukseklik==240){
   irtifaSabitle();
   delay(10000);
  }
  else if(190>yukseklik>=20)
   landKomutu();
   //hız değişkeni
   
  else if(yukseklik==0 && bayrak==2){
   butondurum = digitalRead(buton);   
   
   if (butondurum = 0){
    digitalWrite(buton,LOW);
    Serial.print("SİSTEM KAPANDI");
    sesliIkaz.beep(1 dk boyunca);
   }
   
   else{ 
    continue;
   }
//telemetri verileri ve görüntü aktarımı 1dk boyunca
}
