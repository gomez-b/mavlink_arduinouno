
//Purpose: The purpose of this code is to establish UART
// communication from the PIXHAWK 2.4.8 to ARDUINO MEGA
#include "mavlink.h"
#include <SoftwareSerial.h>
#include <SPI.h>
#include "SdFat.h"

static const uint8_t arduinoRxPin = 10; // Define your receive pin
static const uint8_t arduinoTxPin = 11; // Define your transmit pin

SoftwareSerial mySerial1(arduinoRxPin, arduinoTxPin); // Create a SoftwareSerial object

// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 1;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;
///**************SD SD SD SD SD SD SD SD SD SD ****************/////////
// SD chip select pin.  Be sure to disable any other SPI devices such as Enet.
const uint8_t chipSelect = SS;

// Interval between data records in milliseconds.
// The interval must be greater than the maximum SD write latency plus the
// time to acquire and write data to the SD to avoid overrun errors.
// Run the bench example to check the quality of your SD card.
const uint32_t SAMPLE_INTERVAL_MS = 500;

// Log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "Data"
//------------------------------------------------------------------------------
// File system object.
SdFat sd;

// Log file.
SdFile file;

// Time in micros for next data record.
uint32_t logTime;
//==============================================================================
// User functions.  Edit writeHeader() and logData() for your requirements.

const uint8_t list = 4;
//==============================================================================
// Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))
//------------------------------------------------------------------------------
void setup() {
  //SD setup
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";  
 // MAVLink interface start
  mySerial1.begin(57600);
  Serial.begin(57600);
  // Wait for USB Serial
  while (!Serial) {
    yield();
  }
  delay(1000);

  Serial.println(F("Type any character to start"));
  while (!Serial.available()) {
    yield();
  }

  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(chipSelect, SD_SCK_MHZ(50))) {
    sd.initErrorHalt();
  }

  // Find an unused file name.
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      error("Can't create file name");
    }
  }
  if (!file.open(fileName, O_WRONLY | O_CREAT | O_EXCL)) {
    error("file.open");
  }
  // Read any Serial data.
  do {
    delay(10);
  } while (Serial.available() && Serial.read() >= 0);

  Serial.print(F("Logging to: "));
  Serial.println(fileName);
  Serial.println(F("Type any character to stop"));

  // Write data header.
  writeHeader();

  // Start on a multiple of the sample interval.
  logTime = millis()+ SAMPLE_INTERVAL_MS;
  //logTime *= 1000UL*SAMPLE_INTERVAL_MS;


}



void loop() {


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


  ///SD SD SD SD SD SD SD 
// Time for next record.
  logTime += SAMPLE_INTERVAL_MS;

  // Wait for log time.
  int32_t diff;
  do {
    diff = millis() - logTime;
  } while (diff < 0);

  // Check for data rate too high.
  if (diff > 10) {
    error("Missed data record");
  }

  logData();

  // Force data to SD and update the directory entry to avoid data loss.
  if (!file.sync() || file.getWriteError()) {
    error("write error");
  }
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
    //Mav_Request_Data();
    num_hbs_pasados++;
    if (num_hbs_pasados >= num_hbs) {
      // Request streams from Pixhawk
      Serial.println("Streams requested!");
      Mav_Request_Data();
      num_hbs_pasados = 0;
    }
  }
  //  String hi =comm_receive();
  //  Serial.println(hi);

  

    if (Serial.available()) {
    // Close file and stop.
    file.close();
    Serial.println(F("Done"));
    while (true) {}
  }

}

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


String comm_receive() {
  String result;  // Create a local String variable to hold the received data
  mavlink_message_t msg;
  mavlink_status_t status;


  while (mySerial1.available() > 0) {
    char c = mySerial1.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle message and build the received data
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&msg, &heartbeat);
            // receivedData += "time_usec: " + String(gps_raw_int.time_usec) + ", ";
            result += "System Status: " + String(heartbeat.system_status) + ", ";
            //Serial.println(gps_raw_int.lat);
            result += "Mavlink Version:" + String(heartbeat.mavlink_version) + "\n";

          }

          break;
      }
    }
  }

  return result;
}
//------------------------------------------------------------------------------
// Log a data record.
String logData() {
  String receivedData = comm_receive(); // Store the result in a variable

  Serial.println(receivedData); // Print the received data to Serial

  String data[list];
  for (uint8_t i = 0; i < list; i++) {
    data[i] = receivedData; // Use the stored result in the data array
  }
  // for (uint8_t i = 0; i < list; i++) {
  //   receivedData += data[i];
  //   if (i < list - 1) {
  //     receivedData += ',';
  //   }
  // }
  // receivedData += '\n';
  // Serial.println(receivedData);

  file.print(receivedData); // Print the data to the file

  return receivedData; // Return the data as a string
}

//------------------------------------------------------------------------------
// Write data header.
void writeHeader() {
  file.print(F("Test"));

  file.println();
}
