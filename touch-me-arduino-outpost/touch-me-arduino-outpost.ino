/**
 * ----------------------------------------------------------------------------
 * This is a MFRC522 library example; see https://github.com/miguelbalboa/rfid
 * for further details and other examples.
 *
 * NOTE: The library file MFRC522.h has a lot of useful info. Please read it.
 *
 * Released into the public domain.
 * ----------------------------------------------------------------------------
 * This sample shows how to read and write data blocks on a MIFARE Classic PICC
 * (= card/tag).
 *
 * BEWARE: Data will be written to the PICC, in sector #1 (blocks #4 to #7).
 *
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

#define NUM_LEDS 16
#define DATA_PIN 4

// Define the array of leds
CRGB leds[NUM_LEDS];

#define WIN_STATE          0x80
#define VALID_STATE        0x40
#define COLOR_FIELD_MASK   0x03
#define PATTERN_FIELD_MASK 0x0C
#define MOTION_FIELD_MASK  0x30

#define RST_PIN         9           // Configurable, see typical pin layout above
#define SS_PIN          10          // Configurable, see typical pin layout above

MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.

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

#define INITIAL_COLOR 0x0
#define INITIAL_PATTERN 0x3
#define INITIAL_MOTION 0x3

byte state = INITIAL_COLOR << 0 | INITIAL_PATTERN << 2 | INITIAL_MOTION << 4;
byte new_state = 0;

unsigned int winTime = 0;
const int winLengthMs = 5000;
byte power_mask = 0xFC;
byte power = 0x03;
byte mission = 0xFF;
//byte mission_complete = 0;

/**
 * Initialize.
 */
void setup() {
    Serial.begin(9600); // Initialize serial communications with the PC
    while (!Serial);    // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
    SPI.begin();        // Init SPI bus
    mfrc522.PCD_Init(); // Init MFRC522 card

    // Prepare the key (used both as key A and as key B)
    // using FFFFFFFFFFFFh which is the default at chip delivery from the factory
    for (byte i = 0; i < 6; i++) {
        key.keyByte[i] = 0xFF;
    }

    Serial.println(F("Scan a MIFARE Classic PICC to demonstrate read and write."));
    Serial.print(F("Using key (for A and B):"));
    dump_byte_array(key.keyByte, MFRC522::MF_KEY_SIZE);
    Serial.println();

    Serial.println(F("BEWARE: Data will be written to the PICC, in sector #1"));

    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
    FastLED.setBrightness(32);

    state = state | VALID_STATE;
}

void checkWinStatus() {
  if(millis() - winTime > winLengthMs)
  {
    state = state & ~(WIN_STATE);
  }
}

/**
 * Main loop.
 */
void loop() {
    // advance leds first
    set_leds(state);
    checkWinStatus();
    FastLED.show();
    FastLED.delay(20);
    
    // Look for new cards
    if ( ! mfrc522.PICC_IsNewCardPresent())
        return;

    // Select one of the cards
    if ( ! mfrc522.PICC_ReadCardSerial())
        return;

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

    // set variables according to data and prepare potential new state
    new_state = 0; // zeroing new_state so we start fresh and not have residuals from last runs
    power_mask = buffer[0];
    power = buffer[1];
    mission = buffer[2];
    //mission_complete = buffer[3];
    Serial.print(F("Read power mask ")); Serial.print(power_mask < 0x10 ? " 0" : " "); Serial.println(power_mask, HEX);
    Serial.print(F("Read power ")); Serial.print(power < 0x10 ? " 0" : " "); Serial.println(power, HEX);
    Serial.print(F("Read mission ")); Serial.print(mission < 0x10 ? " 0" : " "); Serial.println(mission, HEX);
    //Serial.print(F("Read mission_valid ")); Serial.print(mission_valid < 0x10 ? " 0" : " "); Serial.println(mission_valid, HEX);
    //Serial.print(F("Read mission_complete ")); Serial.print(mission_complete < 0x10 ? " 0" : " "); Serial.println(mission_complete, HEX);
    new_state = state & power_mask;
    //Serial.println(new_state, HEX);
    new_state = new_state | power | VALID_STATE;
    //Serial.println(new_state, HEX);
    
    // Show the whole sector as it currently is
    //Serial.println(F("Current data in sector:"));
    //mfrc522.PICC_DumpMifareClassicSectorToSerial(&(mfrc522.uid), &key, sector);
    //Serial.println();

    // shut off leds so there is a visual indication for a successful read
    fill_solid(leds, NUM_LEDS, CHSV(0, 0, 64));
    FastLED.show();

    // Handle logic cases:
    // if mission is already pre achieved, apply new_state with win state
    // if new state achieves mission, write mission complete and then change state to win state
    // new state does not achieve mission, apply new state.
    if (mission >= (WIN_STATE | VALID_STATE)) {
      Serial.println(F("mission PRE accomplished, applying state as new state with win state!"));
      state = new_state | WIN_STATE;
      winTime = millis();
    }
    else if (new_state == mission) {
      Serial.println(F("mission ACCOMPLISHED, writing completion bit!"));
      //copy read block to write block so we only change what we meant
      for (byte i = 0; i < 16; i++) {
        dataBlock[i] = buffer[i];
      }
      //set data block mission byte with mission accomplished bit
      new_state = new_state | WIN_STATE;
      dataBlock[2] = new_state;
      write_success = write_and_verify(blockAddr, dataBlock, buffer, size);
      if (!write_success) {
        Serial.println(F("write failed, keeping previous state"));
      }
      else {
        Serial.println(F("write worked, applying new state"));
        state = new_state;
        winTime = millis();
      }
    }
    else {  // mission is not acheived, no need for write, just apply changes
      Serial.println(F("mission not achieved, applying new state"));
      state = new_state;
    }
    
    delay(100);
    set_leds(state);
    Serial.print(F("current state is ")); Serial.print(state < 0x10 ? " 0" : " "); Serial.println(state, HEX);

    // Dump the sector data
    //Serial.println(F("Current data in sector:"));
    //mfrc522.PICC_DumpMifareClassicSectorToSerial(&(mfrc522.uid), &key, sector);
    //Serial.println();

    // Halt PICC
    mfrc522.PICC_HaltA();
    // Stop encryption on PCD
    mfrc522.PCD_StopCrypto1();
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
  CHSV ledsCHSV[NUM_LEDS];
  // state over 127 means WIN_STATE was reached, play victory sequence
  if (state > 127) {
    {
      fill_rainbow(leds, NUM_LEDS, beat8(60), 256 / NUM_LEDS);
      if(random8() < 64) {
        leds[random(NUM_LEDS)] = CRGB::White;
      }
    }
    return;
  }
  // after special cases state can be checked for COLOR, PATTERN and MOTION and set leds accordingly
  switch (state & COLOR_FIELD_MASK) {
    case 0x00 :
      fill_solid(ledsCHSV, NUM_LEDS, CHSV(0 * 256 / 4, 255, 255));
      break;
    case 0x01:
      fill_solid(ledsCHSV, NUM_LEDS, CHSV(1 * 256 / 4, 255, 255));
      break;
    case 0x02:
      fill_solid(ledsCHSV, NUM_LEDS, CHSV(2 * 256 / 4, 255, 255));
      break;
    case 0x03:
      fill_solid(ledsCHSV, NUM_LEDS, CHSV(3 * 256 / 4, 255, 255));
      break;
  }
  switch ( (state & MOTION_FIELD_MASK) >> 4) {
    case 0x00:
      {
        // blink
        uint8_t brightness = beatsin8(30, 64, 255);
        for(int i=0; i<NUM_LEDS; i++) {
          ledsCHSV[i].val = brightness;
        }
      }
      break;
    case 0x01:
      {
        uint8_t snakeHeadLoc = beat8(30) / NUM_LEDS;
        for (byte i=0; i < NUM_LEDS; i++) {
          uint8_t distanceFromHead = (i - snakeHeadLoc + NUM_LEDS) % NUM_LEDS;
          const int snakeLength = 8;
          uint8_t brightness = distanceFromHead > snakeLength ? 255 : 255 - ((int)distanceFromHead * (256 / snakeLength));
          ledsCHSV[i].val = brightness;
        }
      }
      break;
    case 0x02:
      // static
      break;
    case 0x03:
      //flicker
      {
        static bool isOn = true;
        if(random8() < (isOn ? 40 : 24))
          isOn = !isOn;
        uint8_t brightness = isOn ? 255 : 0;
        for(int i=0; i<NUM_LEDS; i++) {
          ledsCHSV[i].val = brightness;
        }
      }
      break;
  }
  switch ((state & PATTERN_FIELD_MASK) >> 2) {
    case 0x00 :
      {
        // Turn off even leds for dotted pattern
        for (byte i=0; i < NUM_LEDS/2; i++) {
          ledsCHSV[i*2].val = 0;
        }
      }
      break;
    case 0x01:
      {
        // Turn off first half
        for (byte i=0; i < NUM_LEDS/2; i++) {
          ledsCHSV[i].val = 0;
        }
      }
      break;
    case 0x02:
      {
         // full
      }
      break;
    case 0x03:
      {
        // Turn off quarters
        for (byte i=0; i < NUM_LEDS/4; i++) {
          ledsCHSV[i].val = 0;
          ledsCHSV[NUM_LEDS/2 + i].val = 0;
        }
      }
      break;
  }
  
  for(int i=0; i<NUM_LEDS; i++) {
    leds[i] = (CRGB)(ledsCHSV[i]);
  }
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
