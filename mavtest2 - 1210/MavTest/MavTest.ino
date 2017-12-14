
#include"mavlink_avoid_errors.h"
#include"MAVLINK/common/mavlink.h"
#include <avr/pgmspace.h>
const int pinNum = 22;
int delayTime=0;
const PROGMEM  uint16_t charSet[]  = { 65000, 32796, 16843, 10, 11234};
mavlink_heartbeat_t  heartbeat;
mavlink_sys_status_t sys;
mavlink_attitude_t   attitude;
mavlink_gps_raw_int_t gpsPos;
mavlink_global_position_int_t position;
mavlink_command_long_t command;
mavlink_command_ack_t ack;
 
unsigned int displayInt;
int k;    // counter variable

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Serial1.begin(57600);
  delayTime = 1;
 
}

void loop() {
  // put your main code here, to run repeatedly:
  
  taskLoop();
  
 // delay(a);
}

void taskLoop()
{
  int a = 5000;
  bool is=false;
   mavlink_message_t msg;
   mavlink_status_t status;
   mavlink_channel_t chan;
    status.packet_rx_drop_count = 0;
  pinMode(pinNum,OUTPUT);
  digitalWrite(pinNum,HIGH);
  //Serial1.print("\t\tReading some bytes: ");
  if(Serial1.available())
  {
    Serial.print("\t\tReading some bytes: ");
    Serial.println(Serial1.available());
  }
  while (Serial1.available() > 0)  
    {
      uint8_t c = uint8_t(Serial1.read()) ;
      is = true;
        //comdata += char(Serial.read());
     if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
     {
        Serial.println(msg.msgid);
        handleMessage(&msg);
        if(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          digitalWrite(pinNum,LOW );
          Serial.println("LOW");
          delay(1000);
          digitalWrite(pinNum,HIGH );
          Serial.println("HIGH");
          delay(a-1000);
          break;
        }
        
     }
        
    }
    
}

void handleMessage(mavlink_message_t* msg)
{//根据Id解析mavlink消息
    //struct Location tell_command = {};                                  // command for telemetry
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_msg_heartbeat_decode(msg, &heartbeat);    
           
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS:{
          mavlink_msg_sys_status_decode(msg,&sys);
          //mavlink_sys_status_decode
          break;
        }
        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_msg_attitude_decode(msg, &attitude);
//            Serial.print("attitude= ");
//            Serial.println(attitude.roll);
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
          mavlink_msg_gps_raw_int_decode(msg,&gpsPos);
          break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_msg_global_position_int_decode(msg, &position);
            break;
        }
//        
//        case MAVLINK_MSG_ID_AHRS: {
//            mavlink_msg_ahrs_decode(msg, &ahrs); 
//            break;
//        }
        
        case MAVLINK_MSG_ID_COMMAND_ACK: {
          mavlink_msg_command_ack_decode(msg,&ack);       
          break;
        }
        default:
            break;
    }     // end switch
    
} // end handle mavlink
