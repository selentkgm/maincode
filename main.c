#include <mavlink.h>
#include <mavlink_helpers.h>
#include <mavlink_types.h>
#include <protocol.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <ezButton.h>
#include <ezBuzzer.h>
#define servoPin 9 //pin

ezButton limitSwitch = 4;//pin
int buzzer_pin = 8;//pin?
Servo servo;
char heartbeat;

ezBuzzer sesliIkaz(buzzer_pin);
ezButton buton(buzzer_pin);

uint8_t mav1[MAVLINK_MAX_PACKET_LEN];
uint8_t mav2[MAVLINK_MAX_PACKET_LEN]; 

int yukseklik   = 0;
int butondurum  = 0; 
int sistemID    = 1;
int companionID = 1;
int otopilotID  = 1;
int bayrak;
int degisken    =0;

void setup(){
  pinMode(buton,INPUT);
  Serial.begin(9600);
  
  limitSwitch.setDebounceTime(50); //50 ms ile kontrol ediliyor
  servo.attach(servoPin);
}
int guidedModu(){//guided
     
    mavlink_command_long_t guided;
    guided.target_system    = 1;
    guided.target_component = 1;
    guided.command          = MAV_CMD_DO_SET_MODE;
    guided.confirmation     = true;
    guided.param1           = 1;
    guided.param2           = 4; //GUIDED
 
   __mavlink_message mesaj;
   mavlink_msg_command_long_encode(sistemID, companionID, &mesaj, &guided);

  
   int gonder = mavlink_msg_to_send_buffer(mesaj);

    return gonder;
}
int armModu(){//arm
     
    mavlink_command_long_t armEtme;     
    armEtme.target_system    = 1;
    armEtme.target_component = 1;
    armEtme.command          = MAV_CMD_COMPONENT_ARM_DISARM;
    armEtme.confirmation     = true;
   
   __mavlink_message mesaj1;
   mavlink_msg_command_long_encode(sistemID, companionID, &mesaj1, &armEtme);

  
   int gonder1= mavlink_msg_to_send_buffer(mesaj1);

    return gonder1;
}
int kalkisModu(){
     
    mavlink_command_long_t kalkis;
    kalkis.target_system    = 1;
    kalkis.target_component = 1;
    kalkis.command          = MAV_CMD_NAV_TAKEOFF;
    kalkis.confirmation     = true;
 
   __mavlink_message mesaj3;
   mavlink_msg_command_long_encode(sistemID, companionID, &mesaj3, &kalkis);

  
   int gonder3= mavlink_msg_to_send_buffer(mesaj3);

     return gonder3;
}

int irtifaSabitle(){
     
    mavlink_command_long_t irtifa;
    irtifa.target_system    = 1;
    irtifa.target_component = 1;
    irtifa.command          = MAV_CMD_NAV_LOITER_TIME;
    irtifa.confirmation     = true;
 
   __mavlink_message mesaj4;
   mavlink_msg_command_long_encode(sistemID, companionID, &mesaj4, &irtifa);

  
   int gonder4= mavlink_msg_to_send_buffer(mesaj4);
     
   
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
     
     int gonder5 = mavlink_msg_to_send_buffer(&mesaj5)
     
     return gonder5;
}

void loop()
{
  sesliIkaz.loop();
  buton.loop();
  
  if(yukseklik==410)
    degisken=1;
  else if(yukseklik==700)
    //1.ayrılma gerçekleşiyor
    
  if(yukseklik==410 && degisken==1){
    bayrak=1;
     
    servo.write(90);   //2.ayrılma gerçekleşti
   }
  delay(1500); //görev yükünün ayrılması için beklenir

  guidedModu();
  armModu();
  kalkisModu(); 
  //hız değişkenleri yapılacak
  
  if(yukseklik==240){
   irtifaSabitle();
   delay(10000);
  }
  else if(190>yukseklik>=20){
   landKomutu();
   //hız değişkeni
  }
  else if(yukseklik==0 && bayrak==2){
   butondurum = digitalRead(buton);   
   
    if (butondurum = 0){
     digitalWrite(buton,LOW);
     Serial.print("SİSTEM KAPANDI");
     sesliIkaz.beep(100000);
   }
   
//telemetri verileri ve görüntü aktarımı 1dk boyunca
} }
