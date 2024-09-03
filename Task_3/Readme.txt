In this task we wish to control a single thruster using Arduino and jetson but such that the
thruster runs by only powering on the jetson and no commands are entered manually. For this we need
to create system service files and enable them so that they run on startup.
We dont see these programs open in the terminal as they run in the background. Controlling using W A S
D keys is not possible as the publisher node terminal where we type it is not available.
So we just publish integer data 1 and run the thruster at constant speed.