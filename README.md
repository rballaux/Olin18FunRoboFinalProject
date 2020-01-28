# Introduction
This repository was used to store the code for the final competition of our Fundamentals of Robotics class at Olin in the fall of 2018.

These are all Arduino sketches to control a boat for four different challenges:

## The challenges

1. Circling the inside perimeter of a round pool.
2. Making figure 8 around two 'iceberg' shaped styrofoam object.
3. Leaving a dock, doing 3 figure 8 shapes in the same way as before and navigate back into the dock. The dock is marked with a red dot above it. This is used to be detected with the pixycam.
4. Follow and try to hit another boat that has a purple narwhal strapped to the top. This object was also detected with the pixycam.


![alt text](https://github.com/adam-p/markdown-here/raw/master/src/common/images/icon48.png "Boat Picture")

## The boat configuration

Our final tugboat sensor system used 6 long range sharp IRs, 3 long range sonar sensors in series and a version 2 pixyCam. There are two maneuverable IRs on each side of the boat and 3 possible positions for each of the two IRs on the front of the boat.
-We used an Arduino mega, extension wings and an Xbee motor shield as micro controller components
-We used 2 7.2V batteries, a marine viper, a DC motor controlling the propellers, and a servo motor controlling the rudders.
