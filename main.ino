//Date: 11/26/2023
#include <TimeLib.h>
//Purpose: The purpose of this code is to establish UART
// communication from the PIXHAWK 2.4.8 to ARDUINO MEGA
#include "mavlink.h"
#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>
const int chipSelect =53;
static const uint8_t arduinoRxPin = 10; // Define your receive pin
static const uint8_t arduinoTxPin = 11; // Define your transmit pin
String dataString;
SoftwareSerial mySerial1(arduinoRxPin, arduinoTxPin); // Create a SoftwareSerial object
unsigned long TimeBetweenReadings = 15000;
unsigned long previousTime = 0;
unsigned long currentTime;
// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 15000;  // next interval to count
const int num_hbs = 20;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;
//For appending info into array
String array[4];
String result[4];

void setup() {
  // MAVLink interface start
  mySerial1.begin(57600);


  Serial.begin(57600);
  Serial.println("MAVLink starting.");
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");

  File dataFile = SD.open("Data7.csv", FILE_WRITE);
  if (!dataFile) {
    Serial.println("Error opening Data1.csv");
  } else {
    Serial.println("Date,Time,Latitude,Longitude");
    dataFile.println("Date,Time,Latitude,Longitude");
    dataFile.close();
  }
  
}

void loop() {
  currentTime = millis();
  if (currentTime - previousTime >= TimeBetweenReadings) {
   previousTime = currentTime;
  // MAVLink
  /* The default UART header for your MCU */
  int sysid = 1;     ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 151;  ///< The component sending the message
  int type = 1;      ///< This system is an airplane / fixed wing


  // Define the system type, in this case an airplane -> on-board controller
  // uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;


  uint8_t system_mode = 0;                   ///< Booting up
  uint32_t custom_mode = 0;                  ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];


  // Pack the message
  //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_pack(1, 0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);


  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);


  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    // Record last HB update
    previousMillisMAVLink = currentMillisMAVLink;

    num_hbs_pasados++;
    if (num_hbs_pasados >= num_hbs) {
      // Request streams from Pixhawk
      Serial.println("Streams requested!");
      Mav_Request_Data();
      num_hbs_pasados = 0;
    //}
 // }
 }
  }
  }
  comm_receive();
  }


//}


void Mav_Request_Data() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];




  // To be setup according to the needed information to be requested from the Pixhawk
  const int maxStreams = 1;
  // const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_POSITION};
  const uint8_t MAVStreams[maxStreams] = { MAV_DATA_STREAM_ALL };
  const uint16_t MAVRates[maxStreams] = { 0x02 };


  for (int i = 0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(1, 151, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mySerial1.write(buf, len);
  }

}


void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;
  String receivedData;
  while (mySerial1.available() > 0) {
    char c = mySerial1.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle message and build the received data
      switch (msg.msgid) {
        // case MAVLINK_MSG_ID_HEARTBEAT:
        //   {
        //     mavlink_heartbeat_t heartbeat;
        //     mavlink_msg_heartbeat_decode(&msg, &heartbeat);
        //     // receivedData += "time_usec: " + String(gps_raw_int.time_usec) + ", ";
        //     result += "System Status: " + String(heartbeat.system_status) + ", ";
        //     //Serial.println(gps_raw_int.lat);
        //     result += "Mavlink Version:" + String(heartbeat.mavlink_version) + "\n";

        //   }

        //   break;
        case MAVLINK_MSG_ID_SYSTEM_TIME:
        {
          mavlink_system_time_t system_time;
          mavlink_msg_system_time_decode(&msg,&system_time);
          uint64_t unixTimeMicroseconds = system_time.time_unix_usec;
          time_t unixTimeSeconds = static_cast<time_t>(unixTimeMicroseconds/ 1000000);
          //Serial.print(unixTimeSeconds);
         // result+= "System Time:"+String(static_cast<unsigned long int> (system_time.time_unix_usec / 1000000));
          //time_t t = result;
          setTime(unixTimeSeconds);
          String date = "Date: " + String(day()-1) + "/" + String(month()) + "/" + String(year()) + " ";
          array[0]=date;
          //Serial.println(array[0]);
          String time = "Time: " + String(hour() + 4) + ":" + String(minute()) + ":" + String(second() )+ ",";
          array[1] = time;
          //Serial.println(array[1]);

        }
        break;

        case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
          mavlink_gps_raw_int_t gps_raw_int;
          mavlink_msg_gps_raw_int_decode(&msg,&gps_raw_int);
          //Serial.println(gps_raw_int.lat);
          String latitude = "Latitude:" + String(gps_raw_int.lat) + ",";
          array[2]=latitude;
          //Serial.println(array[2]);

          String longitude = "Longitude:" + String(gps_raw_int.lon) + "\n";
          array[3] = longitude; 
          //Serial.println(array[3]);

        }
        break;
      }
        // Add corresponding elements of array1 and array2
 // for (int i = 0; i < 6; i++) {
   // receivedData += array[i];
  }
  //   // Print the result
  // Serial.print("Result: ");
  // for (int i = 0; i < 6; i++) {
  //   Serial.print(result[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();


    
  }

  // Log data directly in comm_receive
  File dataFile = SD.open("Data7.csv", FILE_WRITE);
  if (dataFile) {
    for (int i = 0; i < 4; i++) {
      dataFile.print(array[i]);
      Serial.print(array[i]);
      //array[i] = "";
    }
    dataFile.println();  // Add a new line after each set of data
    dataFile.close();    // Clear the array elements after logging all messages
    // for (int i = 0; i < 4; i++) {
    //   array[i] = "";
    //}
  } else {
    Serial.println("Error opening Data1.csv");
  }
//  return received;
}
