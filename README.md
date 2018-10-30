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

The game assumes 3 bytes of information on the RFID tags according to the following mapping:

byte1: power mask - 1's for bits to be left unchanged, 0's for bits to change (bits 6 and 7 should be 1's as to not change mission validity and completeness)

byte2: power - each 2 bits control a different power parameter, 

bits <0-1> color parameter

bits <2-3> pattern parameter

bits <4-5> motion parameter 

bits 6 and 7 effect mission validity and completeness and should not be set using the power byte.

byte3: mission - the total set of values to be matched at bits <0-5>, bit 6 signifys a valid mission is encoded and bit 7 signifys the mission has been completed.
