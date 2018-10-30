# touch-me-arduino
This project contains the Arduino files for an RFID based set matching game.

The game assumes 3 objects:
1. RFID tags, each encoded with a "Power" and capable of carrying a "Mission".
   Power means a value to which a field changes when the RFID is presented.
   A Mission is a set of field values to be matched.
2. A Main station where missions are assigned and encoded to RFID tags 
   the Main station also checks tags for a mission complete flag rewarding with a "Prize".
3. Outposts, locations where the tags change field values according to their encoded Power
   Also if the whole set of the outpost values match the tag encoded mission a complete mission flag is encoded to the tag
   this enables returning to the Main station to collect the Prize.
   
The included Arduino files match the above objects

touch-me-arduino-writer - intended for encoding RFID tags with their initial Power

touch-me-arduino-main - the code for the Main station, also sending tag information over ethernet

touch-me-arduino-outpost - code for an Outpost station
