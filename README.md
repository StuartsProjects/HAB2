The Programs for the HAB2 tracker need the LoRaTracker Library installed see here;

https://github.com/LoRaTracker/LoRaTracker-Library

The programs have been tested against version 1.8.5 of the Arduino IDE. Note that this 
version of the IDE has the -permissive attribute set for the compiler, this allows the 
complier to tolerate some functions that previous versions (that did not have the -permissive
attribute set as default) could not. As many as the complier warnings as possible have been
removed, but these warnings should not prevent the programs compliing on version 1.8.5 of the IDE.

There is a complier warning against the Flash-5 Library, but it only a warning, not an error. 

The programs may not compile of old versions of the complier due to space issues, it was not 
practical to test the programs against each version of the Arduino IDE. 