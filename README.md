# Security_Door_ES_AGH
Assignment project for Embedded Systems course on 1st year of Automatics&amp;Robotics master engineer major, Cyber-Physical-Security specalization; on AGH.


Advanced security mechanism for a restricted area

Goal of this project is to construct a system which will control and oversee the work of physical security mechanisms, guarding the access to a restricted area. Those mechanisms include: alarm siren, armored door lock, two steel covers with their own position sensors and actuators controling the movement.

Most important functions:
Granting access to individuals who input the correct code on dial next to the door for 10 seconds, by disabling the lock, (lock is electromagnetic so this will be done by cutting off power from the lock)

Turning on the alarm siren and closing both curtains (each with their own actuators, position sensors and PD controller), in case of inputting the incorrect code for more than 3 times.

Shutting off the alarm and raising the curtains when special alarm disabling code is input (doesn't open the door lock).

USART communication with PC unit allowing to change parameters of PD controll and security codes, as well as overseeing the variables in the system.


