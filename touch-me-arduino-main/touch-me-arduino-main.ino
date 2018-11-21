/*
 * --------------------------------------------------------------------------------------------------------------------
 * The Main station sketch for the RFID set match game
 * --------------------------------------------------------------------------------------------------------------------
 * This sketch uses the MFRC522 library; for further details see: https://github.com/miguelbalboa/rfid

 * @license Released into the public domain.
 * 
 * Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
 *             Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
 * SPI SS      SDA(SS)      10            53        D10        10               10
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
 * SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
 * SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
 *
 */

#include <SPI.h>
#include <MFRC522.h>
#include <FastLED.h>
#include <Ethernet.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xDE };
IPAddress localIp(10, 0, 0, 222);
IPAddress serverIp(255, 255, 255, 255);
IPAddress TCPserverIp(10, 0, 0, 29);

unsigned int localPort = 8888;      // local port to listen on

#define RING_LEDS 16
#define RINGS     4
#define NUM_LEDS (RING_LEDS*RINGS)
#define DATA_PIN 4

// Define the array of leds
CRGB leds[NUM_LEDS];

enum LedsState { Off, Pattern, Mission };

#define WIN_STATE          0x80
#define VALID_STATE        0x40
#define COLOR_FIELD_MASK   0x03
#define PATTERN_FIELD_MASK 0x0C
#define NUMBER_FIELD_MASK  0x30

#define NO_MSG 0
#define TAG_INFO_MSG 1
#define TAG_RESPONSE_MSG 2
#define WRITE_STATUS_MSG 3
#define SHOW_LEDS_MSG 4
#define HEARTBEAT_MSG 5

#define WIN_AND_ERASE   1
#define NEW_MISSION     2
#define WIN_NO_ERASE    3
#define DISPLAY_MISSION 4

#define RST_PIN         9          // Configurable, see typical pin layout above
#define SS_PIN          8          // Configurable, see typical pin layout above

MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance

MFRC522::MIFARE_Key key;
//MFRC522::StatusCode status;
byte sector         = 1;
byte blockAddr      = 4;
byte dataBlock[]    = {
        0x01, 0x02, 0x03, 0x04, //  1,  2,   3,  4,
        0x05, 0x06, 0x07, 0x08, //  5,  6,   7,  8,
        0x09, 0x0a, 0xff, 0x0b, //  9, 10, 255, 11,
        0x0c, 0x0d, 0x0e, 0x0f  // 12, 13, 14, 15
    };
byte trailerBlock   = 7;
byte buffer[18];
byte size = sizeof(buffer);
bool read_success, write_success, auth_success;

#define INITIAL_COLOR 0x2
#define INITIAL_PATTERN 0x2
#define INITIAL_NUMBER 0x3

byte state = INITIAL_COLOR << 0 | INITIAL_PATTERN << 2 | INITIAL_NUMBER << 4;
LedsState master_state = Pattern;
byte message_type = NO_MSG;
// unsigned long winTime = 0;
const int winLengthMs = 5000;
byte power_mask = 0xFC;
byte power = 0x03;
byte mission = 0xFF;
byte mission_command = 0;
unsigned long heartbeat_time = 0;
byte PICC_version;

unsigned int readCard[4];

#include "RFIDServerComm.h"
#include "touch-me-arduino.h"

void setup() {
    Serial.begin(9600); // Initialize serial communications with the PC
    while (!Serial);    // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
    SPI.begin();        // Init SPI bus
    mfrc522.PCD_Init(); // Init MFRC522 card
    mfrc522.PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details
    Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
    // Prepare the key (used both as key A and as key B)
    // using FFFFFFFFFFFFh which is the default at chip delivery from the factory
    for (byte i = 0; i < 6; i++) {
        key.keyByte[i] = 0xFF;
    }

    Ethernet.begin(mac, localIp);
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
    FastLED.setBrightness(32);

    state = state | VALID_STATE;
}

RFIDServerComm rfidServerComm(TCPserverIp);


/**
 * Main loop.
 */
void loop() {
    // advance leds first
//    if (master_state == Off) {
//      Serial.println("Off");
//    }
//    else if (master_state == Pattern) {
//      Serial.println("Pattern");
//    }
//    else {
//      Serial.println("Mission");
//    }
    set_leds(state, master_state);
    //checkWinStatus();
    FastLED.show();
    FastLED.delay(20);

    // read the RFID reader version to send over the heartbeat as data
    PICC_version = 0;
    PICC_version = mfrc522.PCD_ReadRegister(MFRC522::VersionReg);
    // check if its heartbeat time
    if ((millis() - heartbeat_time) > 1000) {
      rfidServerComm.handle_socket_heartbeat();
      heartbeat_time = millis();
    }

    // check for show leds messages
    message_type = rfidServerComm.handle_socket();
    if (message_type == SHOW_LEDS_MSG) {
      if (state) {
        master_state = Pattern;
        Serial.println("Show leds on");
      }
      else {
        master_state = Off;
        Serial.println("Show leds off");
      }
    }

    // START RFID HANDLING
    // Look for new cards
    if ( ! mfrc522.PICC_IsNewCardPresent())
        return;

    // Select one of the cards
    if ( ! mfrc522.PICC_ReadCardSerial())
        return;
	 
	  // Dump debug info about the card; PICC_HaltA() is automatically called
    //	mfrc522.PICC_DumpToSerial(&(mfrc522.uid));

    // get card uid
    Serial.print("found tag with ID: ");
    for (int i = 0; i < mfrc522.uid.size; i++) {  // for size of uid.size write uid.uidByte to readCard
      readCard[i] = mfrc522.uid.uidByte[i];
      Serial.print(readCard[i], HEX);
    }
    Serial.println();
    
    // get PICC card type
    // Serial.print(F("PICC type: "));
    MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
    // Serial.println(mfrc522.PICC_GetTypeName(piccType));

    // Check for compatibility
    if (    piccType != MFRC522::PICC_TYPE_MIFARE_MINI
        &&  piccType != MFRC522::PICC_TYPE_MIFARE_1K
        &&  piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
        Serial.println(F("This sample only works with MIFARE Classic cards."));
        return;
    }

    // perform authentication to open communication
    auth_success = authenticate(trailerBlock, key);
    if (!auth_success) {
      Serial.println(F("Authentication failed"));
      return;
    }

    // read the tag to get coded information
    read_success = read_block(blockAddr, buffer, size);
    if (!read_success) {
      Serial.println(F("Initial read failed, closing connection"));
      // Halt PICC
      mfrc522.PICC_HaltA();
      // Stop encryption on PCD
      mfrc522.PCD_StopCrypto1();
      return;
    }
    // update local variables according to what was read from card
    power_mask = buffer[0];
    power = buffer[1];
    mission = buffer[2];

    // after we read the tag send its info to the server and get a response
    rfidServerComm.handle_socket_tag();

    // if response requires it, write to the tag
    if (mission_command) {
      if (mission_command == WIN_NO_ERASE) {
        Serial.println(F("mission accomplished, play win state without clearing it"));
        master_state = Mission;
        state = WIN_STATE;
      }
      else if (mission_command == DISPLAY_MISSION) {
        Serial.println(F("mission valid but not complete, display it"));
        master_state = Mission;
        state = mission;
      }
      else {
        // try to write mission to tag
        for (byte i = 0; i < 16; i++) {
          dataBlock[i] = buffer[i];
        }
        dataBlock[2] = mission;
        write_success = write_and_verify(blockAddr, dataBlock, buffer, size);
  
        rfidServerComm.handle_socket_write_status(write_success);
  
        if (write_success) {
          Serial.println(F("write worked"));
          if (mission_command == WIN_AND_ERASE) {
            Serial.println(F("mission accomplished, play win state"));
            master_state = Mission;
            state = WIN_STATE;
            // winTime = millis();
          }
          else if (mission_command == NEW_MISSION) {
            Serial.println(F("display new mission to be written to tag"));
            master_state = Mission;
            state = mission;
          }
        }
        else {
          Serial.println(F("write failed"));
          // Halt PICC
          mfrc522.PICC_HaltA();
          // Stop encryption on PCD
          mfrc522.PCD_StopCrypto1();
          return;
        }
      }
    }

    // Dump the sector data, good for debug
    //Serial.println(F("Current data in sector:"));
    //mfrc522.PICC_DumpMifareClassicSectorToSerial(&(mfrc522.uid), &key, sector);
    //Serial.println();

    // Halt PICC
    mfrc522.PICC_HaltA();
    // Stop encryption on PCD
    mfrc522.PCD_StopCrypto1();

    // visual indication for a successful tag operation read or write, failed writes dont get here
    fill_solid(leds, NUM_LEDS, CHSV(0, 0, 64));
    FastLED.show();
    delay(100);
    
    //set_leds(state, master_state);
    //Serial.print(F("current state is ")); Serial.print(state < 0x10 ? " 0" : " "); Serial.println(state, HEX);
    
    //delay(1000);
}
