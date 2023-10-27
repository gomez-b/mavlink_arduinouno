//Author: Brian Gomez Jimenez
#include "mavlink.h"
#include <SoftwareSerial.h>
SoftwareSerial Serial1(9, 10); // PIN 9=Telemetry TX->Pixhawk RX, PIN 10=Telemetry RX->Pixhawk TX

// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 15;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;

void setup() {
  // MAVLink interface start
  Serial1.begin(57600);

  Serial.begin(57600);
  Serial.println("MAVLink starting.");
}

void loop() {

  // MAVLink
  /* The default UART header for your MCU */
  int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 151;                ///< The component sending the message
  int type = 1;   ///< This system is an airplane / fixed wing

  // Define the system type, in this case an airplane -> on-board controller
  // uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

  uint8_t system_mode = 0; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

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
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink)
  {
    // Record last HB update
    previousMillisMAVLink = currentMillisMAVLink;


    //Mav_Request_Data();
    num_hbs_pasados++;
    if (num_hbs_pasados >= num_hbs) {
      // Request streams from Pixhawk
      Serial.println("Streams requested!");
      Mav_Request_Data();
      num_hbs_pasados = 0;
    }

  }

  // Check reception buffer
  comm_receive();
  //delay(500);
}


void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];


  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 1;
  //  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_POSITION};
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_ALL};
  const uint16_t MAVRates[maxStreams] = {0x02};

  for (int i = 0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(1, 151, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}

void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;
  static bool messageProcessed = true;  // Initialize the flag

  int chan = 0;
  while (Serial1.available() > 0) {
    uint8_t c = Serial1.read();

    // Try to get a new message "mavlink_helpers.h"
    if (mavlink_parse_char(MAVLINK_COMM_0,c,&msg,&status))
    {
      if(!messageProcessed){
      Serial.print("Received message with ID ");
      Serial.print(msg.msgid);
      Serial.print(", sequence: ");
      Serial.print(msg.seq);
      Serial.print(" from component ");
      Serial.print(msg.compid);
      Serial.print(" of system ");
      Serial.println(msg.sysid);
      }

      // Handle message
      switch (msg.msgid) {
        // case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
        //   {
        //     // E.g. read GCS heartbeat and go into
        //     // comm lost mode if timer times out
        //     mavlink_heartbeat_t heartbeat;
        //     mavlink_msg_heartbeat_decode(&msg, &heartbeat);
        //     Serial.println("Receiving heartbeats:");
        //     Serial.println(heartbeat.system_status);
        //     Serial.print("Type of MAV:");
        //     Serial.println(heartbeat.type);
        //   }
        //   break;

        // case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
        //   {
        //     /* Message decoding: PRIMITIVE
        //           mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
        //     */
        //     //mavlink_message_t* msg;
        //     //mavlink_sys_status_t sys_status;
        //   mavlink_sys_status_t sys_status;
        //   mavlink_msg_sys_status_decode(&msg, &sys_status);

        //   // Display battery_remaining, voltage_battery, and current_battery
        //   Serial.print("Battery Remaining: ");
        //   Serial.print(sys_status.battery_remaining);
        //   Serial.println("%");
          
        //   Serial.print("Voltage Battery: ");
        //   Serial.print(sys_status.voltage_battery);
        //   Serial.println(" mV");
          
        //   Serial.print("Current Battery: ");
        //   Serial.print(sys_status.current_battery);
        //   Serial.println(" mA");
   
        //   }
        //   break;


        case MAVLINK_MSG_ID_GPS_RAW_INT:  // #24 GPS_RAW_INT
          {
            // "mavlink_msg_gps_raw_int.h"
            mavlink_gps_raw_int_t gps_raw_int;
            mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);
            Serial.print("time_usec: ");
            Serial.print(static_cast<uint32_t>(gps_raw_int.time_usec));

            Serial.print(", Latitude: ");
            Serial.print(gps_raw_int.lat);

            Serial.print(", Longitude: ");
            Serial.println(gps_raw_int.lon);
          }
          break;

        case MAVLINK_MSG_ID_SYSTEM_TIME: //2 SYSTEM_TIME
        {
        // "mavlink_msg_system_time.h"
            mavlink_system_time_t system_time;
            mavlink_msg_system_time_decode(&msg,&system_time);
            Serial.print("Timestamp of master clock in UNIX epoch:");
            Serial.println(static_cast<uint32_t>(system_time.time_unix_usec));
        }
        break;

      }
    }
    //delay(5000);
  }
  //delay(1000);
}