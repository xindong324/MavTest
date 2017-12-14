#include"define.h"
#include "global.h"
#include"mavlink_avoid_errors.h"
#include"MAVLINK/common/mavlink.h"
#include <avr/pgmspace.h>
#include <SPI.h>
#include <SD.h>


const int chipSelect = 4;
const int pinNum = 22;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Serial1.begin(57600);
  delayTime = 1;
  delay(100);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
  }
  else{
   CreatFile();
  }
  
 
}

void loop() {
  // put your main code here, to run repeatedly:
  //send_heartbeat(MAVLINK_COMM_0);
  taskLoop();
  //delay(100);
  Serial.flush();
  Serial1.flush();
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
      //Serial.println(Serial1.read());
      uint8_t c = uint8_t(Serial1.read()) ;
      is = true;
        //comdata += char(Serial.read());
     if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
     {
        Serial.println(msg.msgid);
        handleMessage(&msg);
        //send_heartbeat(MAVLINK_COMM_0);
        if(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
         // digitalWrite(pinNum,LOW );
          Serial.println("LOW");
          //delay(1000);
         // digitalWrite(pinNum,HIGH );
         // Serial.println("HIGH");
         // delay(a-1000);
          break;
        }
        
     }
        
    }
    
}

void RecordLog()
{
  File myFile = SD.open(FileName, FILE_WRITE);
  myFile.close();
}

void CreatFile()
{
  int FileNum = 0;
  // if the file is available, write to it:
  File Dir = SD.open("/dxx/");
  if(Dir.isDirectory() ==false)
  {
    Serial.println("Not a Dir");
  }
   FileNum = CountFile(Dir);
  Serial.println(FileNum);
   FileName = "";
  if(FileNum<0)
  {
    SD.mkdir("dxx");
    FileName += "dxx/";
  }
   FileName += "DataLog"+String(FileNum+1)+".txt";
 
}

int CountFile(File Dir)
{
  int Count = 0;
   File tDir = Dir;
  
  while(true)
  {
    File entry = Dir.openNextFile();
    if(! entry) break;
    
   // Serial.println(Dir.name());
    if(entry.isDirectory()) continue;
    else Count++;
    entry.close();
  }
  return Count;
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
        case MAVLINK_MSG_ID_SYSTEM_TIME:{
          mavlink_msg_system_time_decode(msg,&sys_time);
          Serial.print("SYS_TIME");
          Serial.println(sys_time.time_boot_ms);
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


//static  void send_heartbeat(mavlink_channel_t chan)
//{
//    mavlink_message_t msgSend={0};
//    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
//    uint8_t system_status = MAV_STATE_ACTIVE;
//    uint32_t custom_mode = control_mode;
//    uint8_t *cse;
//    // work out the base_mode. This value is not very useful
//    // for APM, but we calculate it as best we can so a generic
//    // MAVLink enabled ground station can work out something about
//    // what the MAV is up to. The actual bit values are highly
//    // ambiguous for most of the APM flight modes. In practice, you
//    // only get useful information from the custom_mode, which maps to
//    // the APM flight mode and has a well defined meaning in the
//    // ArduPlane documentation
//    base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
//    switch (control_mode) {
//    case AUTO:
//    case RTL:
//    case LOITER:
//    case GUIDED:
//    case CIRCLE:
//        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
//        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
//        // APM does in any mode, as that is defined as "system finds its own goal
//        // positions", which APM does not currently do
//        break;
//    }
//    // all modes except INITIALISING have some form of manual
//    // override if stick mixing is enabled
//    base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
//#if HIL_MODE != HIL_MODE_DISABLED
//    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
//#endif
//    // we are armed if we are not initialising
//    if (0){//motors.armed()) {
//        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
//    }
//    // indicate we have set a custom mode
//    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
////    dxxmavlink_msg_heartbeat_send(
////        chan,
////        MAV_TYPE_QUADROTOR,
////        MAV_AUTOPILOT_ARDUPILOTMEGA,
////        base_mode,
////        custom_mode,
////        system_status);
//        
//    mavlink_msg_heartbeat_pack(
//        1,
//        0,
//        &msgSend,
//        MAV_TYPE_QUADROTOR,
//        MAV_AUTOPILOT_ARDUPILOTMEGA,
//        base_mode,
//        custom_mode,
//        system_status);
//        byte b[sizeof(msgSend)];
//       // memcpy(b,&msgSend,sizeof(msgSend));
//       // Serial1.write(b,sizeof(msgSend));
//       // delay(500);
//        //memcpy(cse,msgSend, strlen(msgSend));
////        mavlink_msg_to_send_buffer(cse,&msgSend);
////        Serial.println(msgSend.sysid);
////        Serial.println(msgSend.msgid);
//        
//        
//}

