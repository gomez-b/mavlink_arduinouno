#include <ezButton.h>

/*** Ping libraries */
#include <ping-message-all.h>
#include <ping-message-common.h>
#include <ping-message-ping1d.h>
#include <ping-message-ping360.h>
#include <ping-message.h>
#include <ping-parser.h>
#include <ping1d.h>
#include "ping1d.h"
/** SD libraries */
#include <SPI.h>
#include <SD.h>
/** Mavlink Libraries  **/
#include "mavlink.h"
#include <SoftwareSerial.h>
#include <TimeLib.h>

#define ARRAY_SIZE 11  // Define the size of the array

#define BUFFER_SIZE 10
float data_buffer[BUFFER_SIZE];

static const uint8_t mavlinkRxPin = 10;                // Rx on PXH -> pin 10 on MCU
static const uint8_t mavlinkTxPin = 11;                // Tx on PXH -> pin 11 on MCU
SoftwareSerial mySerial1(mavlinkRxPin, mavlinkTxPin);  // Create a SoftwareSerial object

/**  UART Serial Communication  **/
//**UNCOMMENT TO USE HARDWARE SERIAL COMMUNICATION**///*
static const uint8_t arduinoRxPin = 19;  // Rx on PXH -> pin 10 on MCU
static const uint8_t arduinoTxPin = 18;  // Tx on PXH -> pin 11 on MCU
static Ping1D ping{ Serial1 };
/*
static const uint8_t arduinoRxPin = 12;                 // White Wire on Sonar to pin 12
static const uint8_t arduinoTxPin = 13;                 // Green Wire on Sonar to pin 13
SoftwareSerial pingSerial(arduinoRxPin, arduinoTxPin);  // Create a SoftwareSerial object
static Ping1D ping{ pingSerial };
*/

//declaring floating variables for Salinity
float Salinity_Analog_Pin = A0;
float Salinity_Voltage;
float Two_point_calibration;

//declaring floating variables for Temp
float Temp_Analog_Pin = A2;
float Temp_Voltage;
float Temp;
float mm_ft_conversion;
//variable to store the resistance of the thermistor
unsigned long thermistor;

unsigned long previousMillisMAVLink = 0;
unsigned long next_interval_MAVLink = 500;
const int num_hbs = 30;  // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;

String data_array[ARRAY_SIZE];
bool program = false;

// variables will change:
const int button = 5;  //this is connected to pin 5 in Digital Section
int count = 0;
int newcount = 0;
int LED_R = 7;  // Red LED
int LED_G = 6;  // Green LED
int LED_B = 4;  // Blue LED
int LED_Y = 3;  // Yellow LED
int LED_W = 2;  // White LED

const int chipSelect = 53;  // Pin connected to SD card's chip select line

unsigned long resistance(unsigned long Temp_Analog);  //function to calculate resistance from Temp
float steinharthart(unsigned long resistance);        //calculates resistance for Steinhart-Hart's equation

void setup() {
  Serial.begin(57600);     // Sets up the serial monitor
  Serial1.begin(57600);    //
  mySerial1.begin(57600);  // Starts Serial communication for Pixhawk

  pinMode(Salinity_Analog_Pin, INPUT);
  pinMode(Temp_Analog_Pin, INPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_W, OUTPUT);


  while (!ping.initialize() || !SD.begin(chipSelect) || !mySerial1.available()) {  // Checks if sonar and SD card are available
    if (!ping.initialize()) {
      Serial.println("\nPing device failed to initialize!");
      Serial.println("Are the Ping RX/TX wired correctly?");
      Serial.print("Ping RX is the green wire, and should be connected to Arduino pin ");
      Serial.print(arduinoTxPin);
      Serial.println(" (Arduino TX)");
      Serial.print("Ping TX is the white wire, and should be connected to Arduino pin ");
      Serial.print(arduinoRxPin);
      Serial.println(" (Arduino RX)");
      Serial.println(" ");
      blinkLedFast(LED_B, 800);  // Toggle LED_B
      delay(1000);
    }

    if (!SD.begin(chipSelect)) {
      Serial.println("SD card initialization failed. Things to check:");
      Serial.println("1. Is a card inserted?");
      Serial.println("2. Is your wiring correct?");
      Serial.println("3. Did you change the chipSelect pin to match your shield or module?");
      Serial.println("Note: Press the reset button on the board and reopen this Serial Monitor after fixing your issue!");
      Serial.println(" ");
      blinkLedFast(LED_Y, 800);  // Toggle LED_Y
      delay(1000);
    }

    if (!mySerial1.available()) {
      Serial.println("Check Pixhawk Wiring");
      blinkLedFast(LED_W, 800);  // Toggle LED_Y
      delay(1000);
    }
  }
  digitalWrite(LED_B, HIGH);  // Turn on LED_B
  digitalWrite(LED_Y, HIGH);  // Turn on LED_Y
  digitalWrite(LED_W, HIGH);  // Turn on LED_W
  Serial.println(" ");
  Serial.println("All devices initialized successfully. Press button to continue.");
}


void startprogram() {
  program = true;
  while (program) {

    readTemperatureAndSalinity();


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
      num_hbs_pasados++;  //LAST THING THAT WAS CHANGED< CURRENTLY LOGGING EVERY 8 seconds
      if (num_hbs_pasados >= num_hbs) {
        Serial.println("Streams requested!");
        readPingData();
        Mav_Request_Data();
        num_hbs_pasados = 0;
      }
    }

    comm_receive();
    if (digitalRead(button) == HIGH) {
      stopprogram();
    }
  }
}
void stopprogram() {
  program = false;
  Serial.println("BUtton is pressed, program has stopped");
}

void loop() {
  // Read the button state
  if (digitalRead(button) == HIGH) {
    // Wait until the button is released to avoid rapid toggling
    while (digitalRead(button) == HIGH) {
      delay(50);
    }
    newcount = count + 1;
    ;
    if (newcount != count) {
      Serial.println(newcount);
      switch (newcount) {
        case 1:
          startprogram();
          break;

        case 2:
          stopprogram();
          break;
      }

      count = newcount;
    }
  }

  // delay (500);
}



void Mav_Request_Data() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const int maxStreams = 1;
  const uint8_t MAVStreams[maxStreams] = { MAV_DATA_STREAM_ALL };
  const uint16_t MAVRates[maxStreams] = { 0x01 };

  for (int i = 0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(1, 151, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mySerial1.write(buf, len);
  }
}

void comm_receive() {

  static bool ready_to_log = false;  // Flag to indicate if all data is received and ready to log

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
            data_array[3] = String(day());
            data_array[4] = String(month());
            data_array[5] = String(year());

            data_array[6] = String(hour() + 4);
            data_array[7] = String(minute());
            data_array[8] = String(second());
          }
          break;
        case MAVLINK_MSG_ID_GPS_RAW_INT:
          {
            mavlink_gps_raw_int_t gps_raw_int;
            mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);
            // Store latitude and longitude in the array
            // float latitude_degree = gps_raw_int.lat;
            // float longitude_degree = gps_raw_int.lon;


            data_array[9] = String(gps_raw_int.lat);
            data_array[10] = String(gps_raw_int.lon);
            if (!ready_to_log) {
              ready_to_log = true;
            } else {
              logDataToSD(data_array[0], data_array[1], data_array[2], data_array[3], data_array[4], data_array[5], data_array[6], data_array[7], data_array[8], data_array[9], data_array[10]);
              ready_to_log = false;  // Reset the flag
            }
          }
          break;
      }
    }
  }
}

void logDataToSD(String current_temp, String current_salinity, String current_depth, String current_day, String current_month, String current_year, String current_hour, String current_minute, String current_second, String current_latitude, String current_longitude) {
  // Open file for writing
  File dataFile = SD.open("data.csv", FILE_WRITE);

  if (dataFile) {
    // Define data labels and values
    String labels[] = { "Temperature", "Salinity", "Depth", "Date", "Time", "Latitude", "Longitude" };
    String values[] = { current_temp, current_salinity, current_depth, current_day + "/" + current_month + "/" + current_year, current_hour + ":" + current_minute + ":" + current_second, current_latitude, current_longitude };

    // Write data to file
    for (int i = 0; i < 7; i++) {
      dataFile.print(labels[i] + ": ");
      dataFile.print(values[i] + " ");
      Serial.print(values[i]);
    }
    dataFile.println();

    // Close the file
    dataFile.close();
    Serial.println(" Data logged to SD card.");
    blinkLedFast(LED_G, 200);
  } else {
    Serial.println("Error opening data file.");
    while (true) {
      blinkLedFast(LED_R, 800);  // Blink for 500 milliseconds
    }
  }
}


void readTemperatureAndSalinity() {
  float Temp_Analog = analogRead(Temp_Analog_Pin);  // reads A2 (Temp)
  thermistor = resistance(Temp_Analog);             // calculate resistance of the thermistor
  Temp = steinharthart(thermistor);                 // calculate temperature using the SteinHart-Hart equation
  Temp_Voltage = Temp_Analog / 1023.0 * 5.0;        // ADC conversion for Temp
                                                    // Serial.println(Temp);
                                                    // delay(1000);
  data_array[0] = String(Temp);

  float Salinity_Analog = analogRead(Salinity_Analog_Pin);       // reads A0 (Salinity)
  Salinity_Voltage = Salinity_Analog / 1023.0 * 5.0;             // ADC conversion for Salinity
  Two_point_calibration = 14.2532 * Salinity_Voltage - 1.65312;  // calibration equation from QuestLab
  // Serial.println(Two_point_calibration);
  // delay(1000);
  data_array[1] = String(Two_point_calibration);
}

void readPingData() {
  //static int buffer_index = 0;  // Static variable to keep track of the current position in the buffer
  if (ping.update()) {
    float mm_ft_conversion = ping.distance() * 0.00328084;  // Convert distance to millimeters and then to feet

    data_array[2] = String(mm_ft_conversion);

  } else {
    Serial.println("No update received");
  }
}



unsigned long resistance(unsigned long Temp_Analog) {
  unsigned long temp;
  temp = (Temp_Analog * 15000) / (1023 - Temp_Analog);
  return temp;
}

float steinharthart(unsigned long resistance) {
  float temp;
  float logRes = log(resistance);
  float k0 = 0.00102119;
  float k1 = 0.000222468;
  float k2 = 0.000000133342;

  temp = 1 / (k0 + k1 * logRes + k2 * logRes * logRes * logRes);
  temp = temp - 273.15;
  return temp;
}
// Function to toggle the LED state rapidly
void blinkLedFast(int ledPin, unsigned long duration) {
  unsigned long blinkInterval = 100;  // Blink interval in milliseconds
  unsigned long startTime = millis();

  while (millis() - startTime < duration) {
    digitalWrite(ledPin, HIGH);  // Turn on the LED
    delay(blinkInterval);        // Wait for the specified interval
    digitalWrite(ledPin, LOW);   // Turn off the LED
    delay(blinkInterval);        // Wait for the specified interval
  }
}
