Controlling depth sensor using jetson Nano connected to arduino.
As the depth rating changes the speed of the thruster changes. The depth value is mapped to a value
between 1200 and 1800.

In this task we take the depth value from depth sensor connected to arduino and send it to
Jetson. The jetson recieves the depth value and sends it back to arduino where it is mapped
between 1200 and 1800.

This could have been done on arduino itself where it reads the depth and controls the speed.
But we here aimed to test whether Jetson can take a value from arduino, do computations on it and
send it back to arduino.

