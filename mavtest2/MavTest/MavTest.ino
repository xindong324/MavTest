#include"define.h"
#include "global.h"
#include"mavlink_avoid_errors.h"
#include"MAVLINK/common/mavlink.h"
#include <avr/pgmspace.h>
#include <SPI.h>
#include <SD.h>
#include<math.h>

int getDelayTime();
const int chipSelect = 4;
const int pinNum = 22;
static int delayTime;
static int initTime=10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Serial1.begin(57600);
//  delayTime = 1;
  delay(100);
  
  
  delayTime = getDelayTime();//得到延迟时间
  lastTime = initTime-1;
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    
  }
  else{
    
   // lastTime = abs(initTime-delayTime);
    CreatFile();//得到应该记录的文件名
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
  bool is=false;
   mavlink_message_t msg;
   mavlink_status_t status;
   mavlink_channel_t chan;
    status.packet_rx_drop_count = 0;
  pinMode(pinNum,OUTPUT);
  digitalWrite(pinNum,HIGH);
  if(Serial1.available())
  {
    Serial.print("\t\tReading some bytes: ");
    Serial.println(Serial1.available());
  }
  while (Serial1.available() > 0)  
    {
      //Serial.println(Serial1.read());
      uint8_t c = uint8_t(Serial1.read());
      is = true;
        //comdata += char(Serial.read());
     if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
     {
        Serial.println(msg.msgid);
        handleMessage(&msg);
        
     }
        
    }
    
}

//打开文件并记录数据,输出低电平
void RecordLog()
{
  static int First = true;
  if((position.time_boot_ms/1000==initTime&&First)||position.time_boot_ms/1000-lastTime >= delayTime)
  {
    
    digitalWrite(pinNum,LOW);
    lastTime = position.time_boot_ms/1000;

    
    
    File myFile = SD.open(FileName, FILE_WRITE);
    String gpsData = String(position.time_boot_ms)+","+String(position.lat)+","+String(position.lon)+","+String(position.relative_alt);
    myFile.println(gpsData);
    myFile.close();
    delay(100);
  
    digitalWrite(pinNum,HIGH);
    
    if(position.time_boot_ms/1000==initTime)
    {
      First = false;
    }
    Serial.print("record gps");
//     Serial.println(position.time_boot_ms/1000);
//     Serial.println(lastTime);
//     Serial.println(delayTime);
    
  }
  
}



//得到需要打开的文件名，此函数并不执行创建或打开文件函数
void CreatFile()
{
  int FileNum = 0;
  // if the file is available, write to it:
  File Dir = SD.open("/data");
  if(Dir.isDirectory() ==false)
  {
    Serial.println("Not a Dir");
    SD.mkdir("data");
  }
   FileNum = CountFile(Dir);
  Serial.println(FileNum);
   FileName = "";
  if(FileNum<0)
  {
    
   
  }else
  {
    Dir.close();
  }
   FileName += "/data/";
   FileName += "Log"+String(FileNum+1)+".txt";
   Serial.println();
 
}


//计算有几个log
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

int getDelayTime()
{
 int  num = 5;
 File dataFile = SD.open("delay.txt");  //打开datalog.txt文件

  if (dataFile) {
    while (dataFile.available()) {  //检查是否dataFile是否有数据
      num = num*10+dataFile.read()-'0';
    }
    Serial.println(num);  //如果有数据则把数据发送到串口
    dataFile.close();  //关闭dataFile
  }  
  
  else {
    num = 5;
   // Serial.println("error opening datalog.txt");  //如果文件无法打开串口发送信息error opening datalog.txt
  }
 
   return num;
}


void handleMessage(mavlink_message_t* msg)
{//根据Id解析mavlink消息
    //struct Location tell_command = {};                                  // command for telemetry
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_msg_heartbeat_decode(msg, &heartbeat);    
           
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_msg_global_position_int_decode(msg, &position);
            if(position.time_boot_ms/1000>=initTime)
            {
               RecordLog();
               Serial.println("record");
            }
            Serial.print("Time: ");
            Serial.println(position.time_boot_ms);
            Serial.print("Lan: ");
            Serial.println(position.lat);
            Serial.print("Lon: ");
            Serial.println(position.lon);
            Serial.print("alt: ");
            Serial.println(position.relative_alt);
            
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

