#Geoaware

Our solution for the GeoAware event in Robotix, Kshitij 2014 : www.robotix.in/events/event/geoaware


##Installation

Dependencies:

1. OpenCV 2.4


##Usage

`cd src
make
./Geoaware <map-location>`

## Structure


* Map, TJunction, Waypoint, Landmark
  The map image is processed and store in an instance of `Map`.
  A map has many `TJunctions` and `Landmarks`.
  Each `Landmark` and `TJunction` is a `Waypoint`

* MapProcessor
  Processes the map image into an instance of `Map` and finds the best possible routes across the maze


* CamController
  Interface with video camera

* LiveSymbolDetector
  Detect symbols using the video stream from `CamController`

* Locomotor
  Interface with the motors

* Controller
  Runs the control loop that uses the `LiveSymbolDetector`, `Map` and `Locomotor` to traverse th maze

* ardunio/
  All arduino-specific code

* SoundPlayer
  Play Mario sounds when sub-goals are reached :P


Sample map images can be found in `assets`


