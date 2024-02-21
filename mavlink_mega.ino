//Tuesday Febuary 20, 2024
#include "mavlink.h"
#include <SoftwareSerial.h>
#include <TimeLib.h>

#define ARRAY_SIZE 4 // Define the size of the array

static const uint8_t arduinoRxPin = 10; // Rx on PXH -> pin 10 on MCU
static const uint8_t arduinoTxPin = 11; // Tx on PXH -> pin 11 on MCU

SoftwareSerial mySerial1(arduinoRxPin, arduinoTxPin); // Create a SoftwareSerial object

unsigned long previousMillisMAVLink = 0;
unsigned long next_interval_MAVLink = 2000;
const int num_hbs = 60;
int num_hbs_pasados = num_hbs;

String data_array[ARRAY_SIZE];
int array_index = 0;

void setup() {
  mySerial1.begin(57600);
  Serial.begin(57600);
  Serial.println("MAVLink starting.");
}

void loop() {
  int sysid = 1;
  int compid = 151;
  int type = 1;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
  uint8_t system_mode = 0;
  uint32_t custom_mode = 0;
  uint8_t system_state = MAV_STATE_STANDBY;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;

  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    previousMillisMAVLink = currentMillisMAVLink;
    num_hbs_pasados++;
    if (num_hbs_pasados >= num_hbs) {
      Serial.println("Streams requested!");
      Mav_Request_Data();
      num_hbs_pasados = 0;
    }
  }

  comm_receive();
 

  if (array_index >= ARRAY_SIZE) {
    print_data_array();
    array_index = 0;
  }
}

void Mav_Request_Data() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const int maxStreams = 1;
  const uint8_t MAVStreams[maxStreams] = { MAV_DATA_STREAM_ALL };
  const uint16_t MAVRates[maxStreams] = { 0x06 };

  for (int i = 0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(1, 151, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mySerial1.write(buf, len);
  }
}

void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (mySerial1.available() > 0) {
    char c = mySerial1.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle message and build the received data
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_SYSTEM_TIME:
          {
            mavlink_system_time_t system_time;
            mavlink_msg_system_time_decode(&msg, &system_time);
            uint64_t unixTimeMicroseconds = system_time.time_unix_usec;
            time_t unixTimeSeconds = static_cast<time_t>(unixTimeMicroseconds / 1000000);
            setTime(unixTimeSeconds);
            // Store date and time in the array
            data_array[0] = String(day() - 1) + "/" + String(month()) + "/" + String(year());
            data_array[1] = String(hour() + 4) + ":" + String(minute()) + ":" + String(second());
          }
          break;
        case MAVLINK_MSG_ID_GPS_RAW_INT:
          {
            mavlink_gps_raw_int_t gps_raw_int;
            mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);
            // Store latitude and longitude in the array
            data_array[2] = String(gps_raw_int.lat);
            data_array[3] = String(gps_raw_int.lon);
          }
          break;
      }
    }
  }

  // If all data is available, print it
  if (data_array[0] != "" && data_array[1] != "" && data_array[2] != "" && data_array[3] != "") {
    print_data_array();
    delay(2000);
  }
}

void print_data_array() {
  // Concatenate the required elements into a single string
  String dataString = "Date: " + data_array[0] + ", Time: " + data_array[1] + ", Latitude: " + data_array[2] + ", Longitude: " + data_array[3];
  
  // Print the concatenated string
  Serial.println(dataString);
}
