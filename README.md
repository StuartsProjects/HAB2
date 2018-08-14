The Programs for the HAB2 tracker need the LoRaTracker Library installed see here;

https://github.com/LoRaTracker/LoRaTracker-Library

The HAB2 tracker board is a minimal component count high altitude balloon tracker. 

The components required to build the tracker are described in the 'HAB2 Minimum Parts Tracker Build.pdf' file 
that will be found in the \Build folder. 

The programs have been tested against version 1.8.5 of the Arduino IDE. Note that this 
version of the IDE has the -permissive attribute set for the compiler, this allows the 
complier to tolerate some functions that previous versions (that did not have the -permissive
attribute set as default) could not. As many as the complier warnings as possible have been
removed, but these warnings should not prevent the programs compliing on version 1.8.5 of the IDE.

There is a complier warning against the Flash-5 Library, but it is only a warning and not an error. 

The programs may not compile on old versions of the complier due to space issues or other changes
that may have occured in the Arduino environment over time, it is not practical to test the 
programs against each particular version of the Arduino IDE. If you have problems, you will need 
to upgrade your Arduino IDE to a modern version. 

The user of these programs is expected to have had a reasonable level of experience in using the
Arduino environment. Using the programs does require the user to make configuration changes so
they will need to be able to cope with complier errors due to issues that they may create. 

Stuart Robinson
www.LoRaTracker.uk

  