/*
 * --------------------------------------------------------------------------------------------------------------------
 * Example sketch/program showing how to read data from a PICC to serial.
 * --------------------------------------------------------------------------------------------------------------------
 * This is a MFRC522 library example; for further details and other examples see: https://github.com/miguelbalboa/rfid
 * 
 * Example sketch/program showing how to read data from a PICC (that is: a RFID Tag or Card) using a MFRC522 based RFID
 * Reader on the Arduino SPI interface.
 * 
 * When the Arduino and the MFRC522 module are connected (see the pin layout below), load this sketch into Arduino IDE
 * then verify/compile and upload it. To see the output: use Tools, Serial Monitor of the IDE (hit Ctrl+Shft+M). When
 * you present a PICC (that is: a RFID Tag or Card) at reading distance of the MFRC522 Reader/PCD, the serial output
 * will show the ID/UID, type and any data blocks it can read. Note: you may see "Timeout in communication" messages
 * when removing the PICC from reading distance too early.
 * 
 * If your reader supports it, this sketch/program will read all the PICCs presented (that is: multiple tag reading).
 * So if you stack two or more PICCs on top of each other and present them to the reader, it will first output all
 * details of the first and then the next PICC. Note that this may take some time as all data blocks are dumped, so
 * keep the PICCs at reading distance until complete.
 * 
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
 */

#include <SPI.h>
#include <MFRC522.h>
#include <FastLED.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xDE };
IPAddress localIp(10, 0, 0, 222);
IPAddress serverIp(255, 255, 255, 255);
//IPAddress serverIp(10, 0, 0, 220);

unsigned int localPort = 8888;      // local port to listen on

EthernetUDP Udp;

#define NUM_LEDS 16
#define DATA_PIN 4

// Define the array of leds
CRGB leds[NUM_LEDS];

#define WIN_STATE          0x80
#define VALID_STATE        0x40
#define COLOR_FIELD_MASK   0x03
#define PATTERN_FIELD_MASK 0x0C
#define MOTION_FIELD_MASK  0x30

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
byte state = 0, new_state = 0;
byte power_mask, power, mission, mission_complete;

unsigned long heartbeat_time = 0;
byte PICC_version;

unsigned int readCard[4];

void setup() {
	Serial.begin(9600);		// Initialize serial communications with the PC
	while (!Serial);		// Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
	SPI.begin();			// Init SPI bus
	mfrc522.PCD_Init();		// Init MFRC522
	mfrc522.PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details
	Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
  // Prepare the key (used both as key A and as key B)
  // using FFFFFFFFFFFFh which is the default at chip delivery from the factory
  for (byte i = 0; i < 6; i++) {
      key.keyByte[i] = 0xFF;
  }

  Ethernet.begin(mac, localIp);
  Udp.begin(localPort);
}

void loop() {
  // advance leds first
  FastLED.show();
  FastLED.delay(20);
  
  PICC_version = 0;
  PICC_version = mfrc522.PCD_ReadRegister(MFRC522::VersionReg);

  if (((millis() - heartbeat_time) > 1000) && (PICC_version == 146)) {
    Serial.print("sending 1 sec keep alive, read Firmware version is: 0x");
    Serial.println(PICC_version, HEX);
    Udp.beginPacket(serverIp, 5007);
    Udp.print(0);
    Udp.endPacket();
    heartbeat_time = millis();
  }
	// Look for new cards
	if ( ! mfrc522.PICC_IsNewCardPresent()) {
		return;
	}

	// Select one of the cards
	if ( ! mfrc522.PICC_ReadCardSerial()) {
		return;
	}

	// Dump debug info about the card; PICC_HaltA() is automatically called
//	mfrc522.PICC_DumpToSerial(&(mfrc522.uid));

  for (int i = 0; i < mfrc522.uid.size; i++) {  // for size of uid.size write uid.uidByte to readCard
    readCard[i] = mfrc522.uid.uidByte[i];
    Serial.print(readCard[i], HEX);
  }

  // Show some details of the PICC (that is: the tag/card)
  Serial.print(F("Card UID:"));
  dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
  Serial.println();
  Serial.print(F("PICC type: "));
  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
  Serial.println(mfrc522.PICC_GetTypeName(piccType));

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

  // set variables according to data
  power_mask = buffer[0];
  power = buffer[1];
  mission = buffer[2];
  mission_complete = buffer[3];
  Serial.print(F("Read power mask ")); Serial.print(power_mask < 0x10 ? " 0" : " "); Serial.println(power_mask, HEX);
  Serial.print(F("Read power ")); Serial.print(power < 0x10 ? " 0" : " "); Serial.println(power, HEX);
  Serial.print(F("Read mission ")); Serial.print(mission < 0x10 ? " 0" : " "); Serial.println(mission, HEX);
  //Serial.print(F("Read mission_valid ")); Serial.print(mission_valid < 0x10 ? " 0" : " "); Serial.println(mission_valid, HEX);
  Serial.print(F("Read mission_complete ")); Serial.print(mission_complete < 0x10 ? " 0" : " "); Serial.println(mission_complete, HEX);

  // show leds so there is a visual indication for a successful read
  for (byte i=0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Green;
  }
  FastLED.show();

  // Handle logic cases:
  // Mission complete! change state to win state
  // Mission valid but incomplete, show mission
  // No mission, just give indication
  if (mission_complete == 0x01) {
    Serial.println(F("mission complete, going to WIN_STATE"));
    state = WIN_STATE;
  }
  else if ((mission & VALID_STATE) != 0x00) {  // mission is valid, show mission
    Serial.println(F("mission is valid, show mission"));
    state = mission;
  }
  else { // no mission loaded
    state = 0x00;
  }
  
  delay(100);
  set_leds(state);
  Serial.print(F("current state is ")); Serial.print(state < 0x10 ? " 0" : " "); Serial.println(state, HEX);

  // Send tag ID and data over ethernet, change to TCP?
  
  Udp.beginPacket(serverIp, 5007);
  Udp.print(readCard[0]);
  Udp.print(readCard[1]);
  Udp.print(readCard[2]);
  Udp.print(readCard[3]);
  Udp.print(power);
  Udp.print(mission);
  Udp.print(mission_complete);

  Udp.endPacket();
  //delay(1000);
}



/**
 * Helper routine to dump a byte array as hex values to Serial.
 */
void dump_byte_array(byte *buffer, byte bufferSize) {
    for (byte i = 0; i < bufferSize; i++) {
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], HEX);
    }
}

void set_leds(byte state) {
  //Serial.println(F("showing leds"));
  // state all zeros means never set.. consider what to have leds do
  if (state == 0x00) {
    for (byte i=0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Black;
    }
    return;
  }
  // state over 127 means WIN_STATE was reached, play victory sequence
  if (state > 127) {
    for (byte i=0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Purple;
    }
    return;
  }
  // after special cases state can be checked for COLOR, PATTERN and MOTION and set leds accordingly
  switch (state & COLOR_FIELD_MASK) {
    case 0x00 :
      for (byte i=0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Green;
      }
      break;
    case 0x01:
      for (byte i=0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Red;
      }
      break;
    case 0x02:
      for (byte i=0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Blue;
      }
      break;
    case 0x03:
      for (byte i=0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Yellow;
      }
      break;
  }
  switch (state & PATTERN_FIELD_MASK) {
    case 0x00 :
      // Turn off even leds for dotted pattern
      for (byte i=0; i < NUM_LEDS/2; i++) {
        leds[i*2] = CRGB::Black;
      }
      break;
    case 0x04:
      // Turn off first half
      for (byte i=0; i < NUM_LEDS/2; i++) {
        leds[i] = CRGB::Black;
      }
      break;
    case 0x0C:
      // Turn off other half
      for (byte i=0; i < NUM_LEDS/2; i++) {
        leds[NUM_LEDS/2 + i] = CRGB::Black;
      }
      break;
    case 0x0F:
      // Turn off quarters
      for (byte i=0; i < NUM_LEDS/4; i++) {
        leds[i] = CRGB::Black;
        leds[NUM_LEDS/2 + i] = CRGB::Black;
      }
      break;
  }
  //switch (state & MOTION_FIELD_MASK) {
  //  case 0x00 :
  //    for (byte i=0; i < NUM_LEDS; i++) {
  //      leds[i] = CRGB::Green;
  //    }
  //    break;
  //  case 0x10:
  //    for (byte i=0; i < NUM_LEDS; i++) {
  //      leds[i] = CRGB::Red;
  //    }
  //    break;
  //  case 0x20:
  //    for (byte i=0; i < NUM_LEDS; i++) {
  //      leds[i] = CRGB::Blue;
  //    }
  //    break;
  //  case 0x30:
  //    for (byte i=0; i < NUM_LEDS; i++) {
  //      leds[i] = CRGB::Yellow;
  //    }
  //    break;
  //}
}

bool authenticate(byte trailerBlock, MFRC522::MIFARE_Key key) {
    MFRC522::StatusCode status;

    // Authenticate using key A
    Serial.println(F("Authenticating using key A..."));
    status = (MFRC522::StatusCode) mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(mfrc522.uid));
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("PCD_Authenticate() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
        return false;
    }
    return true;
}

bool read_block(byte blockAddr, byte buffer[], byte size) {
    MFRC522::StatusCode status;
    
    // Read data from the block
    Serial.print(F("Reading data from block ")); Serial.print(blockAddr);
    Serial.println(F(" ..."));
    status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
        return false;
    }
    Serial.print(F("Data in block ")); Serial.print(blockAddr); Serial.println(F(":"));
    dump_byte_array(buffer, 16); Serial.println();
    Serial.println();
    return true;
}

bool write_and_verify(byte blockAddr, byte dataBlock[], byte buffer[], byte size) {
    MFRC522::StatusCode status;

    // Authenticate using key A
    //Serial.println(F("Authenticating using key A..."));
    //status = (MFRC522::StatusCode) mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(mfrc522.uid));
    //if (status != MFRC522::STATUS_OK) {
    //    Serial.print(F("PCD_Authenticate() failed: "));
    //    Serial.println(mfrc522.GetStatusCodeName(status));
    //    return false;
    //}
    
    // Write data to the block
    Serial.print(F("Writing data into block ")); Serial.print(blockAddr);
    Serial.println(F(" ..."));
    dump_byte_array(dataBlock, 16); Serial.println();
    status = (MFRC522::StatusCode) mfrc522.MIFARE_Write(blockAddr, dataBlock, 16);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Write() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
        return false;
    }
    Serial.println();

    // Read data from the block (again, should now be what we have written)
    Serial.print(F("Reading data from block ")); Serial.print(blockAddr);
    Serial.println(F(" ..."));
    status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
        return false;
    }
    Serial.print(F("Data in block ")); Serial.print(blockAddr); Serial.println(F(":"));
    dump_byte_array(buffer, 16); Serial.println();

    // Check that data in block is what we have written
    // by counting the number of bytes that are equal
    Serial.println(F("Checking result..."));
    byte count = 0;
    for (byte i = 0; i < 16; i++) {
        // Compare buffer (= what we've read) with dataBlock (= what we've written)
        if (buffer[i] == dataBlock[i])
            count++;
    }
    Serial.print(F("Number of bytes that match = ")); Serial.println(count);
    if (count == 16) {
        Serial.println(F("Success :-)"));
        return true;
    } else {
        Serial.println(F("Failure, no match :-("));
        Serial.println(F("  perhaps the write didn't work properly..."));
        Serial.println();
        return false;
    }
}
