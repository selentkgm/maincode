#include <checksum.h>
#include <mavlink_conversions.h>
#include <mavlink_get_info.h>
#include <mavlink_helpers.h>
#include <mavlink_sha256.h>
#include <mavlink_types.h>
#include <protocol.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <ezButton.h>
#include <ezBuzzer.h>
#define servoPin 9 //pin

ezButton limitSwitch //pin
int buzzer_pin = //pin
Servo servo;

ezBuzzer sesliIkaz(buzzer_pin);

unit8_t sistemTuru   = MAV_TYPE_HEXAROTOR;
unit8_t otopilotTuru =  MAV_AUTOPILOT_PIXHAWK;
unit8_t sistemModu   = MAV_MODE_AUTO_ARMED;
unit8_t sistemDurumu = MAV_STATE_STANDBY; 

uint8_t mav1[MAVLINK_MAX_PACKET_LEN];
uint8_t mav2[MAVLINK_MAX_PACKET_LEN]; 

int yukseklik;
int buton = ?;               
int butondurum  = 0; 
int sistemID    = 1;
int companionID = 1;
int otopilotID  = 1;
int bayrak;

void setup(){
  Serial.begin(9600);
  pinMode(buton,INPUT);

  heartbeat.sistemModu   =  MAV_MODE_AUTO_ARMED; 
  heartbeat.custom_mode  = --;// ' mod yaz'
  heartbeat.sistemDurumu = MAV_S TATE_STANDBY;
  limitSwitch.setDebounceTime(50); 50 ms ile kontrol ediliyor
  servo.attach(servoPin);
}
int guidedModu(){//guided
     
    mavlink_command_long_t guided;
    guided.target_system    = 1;
    guided.target_component = 1;
    guided.command          = MAV_CMD_DO_SET_MODE;
    guided.confirmation     =true;
    guided.param1           =1;
    guided.param2           =GUIDED; //4
 
   mavlink_message_t mesaj;
   mavlink_msg_command_long_encode(sistemID, companionID, &mesaj, &guided);

  
   int gonder= write_message(mesaj);

    return gonder;
}
int armModu(){//arm
     
    mavlink_command_long_t armEtme;     armEtme.target_system = 1;
    armEtme.target_component = 1;
    armEtme.command          = MAV_CMD_COMPONENT_ARM_DISARM;
    armEtme.confirmation     =true;
   
   mavlink_message_t mesaj1;
   mavlink_msg_command_long_encode(sistemID, companionID, &mesaj1, &armEtme);

  
   int gonder1= write_message(mesaj1);

    return gonder1;
}
int kalkisModu(){
     
    mavlink_command_long_t kalkis;
    kalkis.target_system    = 1;
    kalkis.target_component = 1;
    kalkis.command          = MAV_CMD_NAV_TAKEOFF;
    kalkis.confirmation     =true;
 
   mavlink_message_t mesaj3;
   mavlink_msg_command_long_encode(sistemID, companionID, &mesaj3, &kalkis);

  
   int gonder3= write_message(mesaj3);

     return gonder3;
}

int irtifaSabitle(){
   if(bayrak==1)
     continue;
     
    mavlink_command_long_t irtifa;
    irtifa.target_system    = 1;
    irtifa.target_component = 1;
    irtifa.command          =  MAV_CMD_NAV_LOITER_TIME;
    irtifa.confirmation     =true;
 
   mavlink_message_t mesaj4;
   mavlink_msg_command_long_encode(sistemID, companionID, &mesaj4, &irtifa);

  
   int gonder4= write_message(mesaj4);
   bayrak=2;
     
   else
     bayrak=1;
   
     return gonder4;
}
int landKomutu(){
 
    mavlink_command_long_t inis;
    inis.target_system    = 1;
    inis.target_component = 1;
    inis.command          = MAV_CMD_NAV_LAND ;
    inis.confirmation     = true;
 
   mavlink_message_t mesaj5;
   mavlink_msg_command_long_encode(sistemID, companionID, &mesaj5, &inis); //1,255,
     
     int gonder5= write_message(mesaj5)
     
     return gonder5;
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

  guidedModu();
  armModu();
  kalkısModu(); 
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
