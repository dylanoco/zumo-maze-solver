# zumo-maze-solver

Variables such as maxSpeed, turnDelay etc. are subject to change depending on the ZUMO and its behaviour.

References for the List, manual Delay() function, Timers and the Turn Sensors at the end.

## Getting Started

After uploading the .ino file to the ZUMO, place the ZUMO in the maze between the two black lines.

Press the 'A' Button on the ZUMO to initiate calibration and to begin traversing the maze.

During the drive, the ZUMO will periodically scan the area for houses.

The ZUMO should make a 'beep' sound when it finds a house, create a 180 degree turn, and continue traversing the maze.

On the second time it detects a house, the ZUMO should beep once again, 180 degree turn, and attempt to return back to its starting position.

Once it is done returning back to its starting position, it should play a tune untill picked up from the maze.

## References

List Library: Found in the ARDUINO IDE Library. https://www.arduino.cc/reference/en/libraries/list/

TurnSensor.h: From the 'RotationResist' example in ARDUINO IDE. File > Examples > ZUMO32u4 > RotationResist

Timer: https://forum.arduino.cc/t/measuring-time/96602

Delay Function Code: https://github.com/arduino/ArduinoCore-avr/blob/eabd762a1edcf076877b7bec28b7f99099141473/cores/arduino/wiring.c#L106-L117

Calibration Code: From the 'LineFollower' example in ARDUINO IDE. File > Examples > ZUMO32u4 > LineFollower




