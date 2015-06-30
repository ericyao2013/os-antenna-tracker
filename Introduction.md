# Introduction #

OSAT was developed because I wanted an antenna tracker that worked with any protocol and I was tired of the proprietary antenna trackers on the market. I wanted an antenna tracker that:

  * Required zero setup
  * Anyone could use and code for
  * Cheap hardware requirements
  * A tracker that would work in stationary and moving environments (on the ground and in the air)
  * Interface with any protocol

# Details #

Much of the code is based on the MultiWii Project code but have simplified it by writing it in Arduino style. I did this because I am not a very good coder and I want as many people to understand it as possible.

Currently supported hardware
  * Hobbyking MultiWii Pro
It would be very easy to expand support to new boards.


Currently supported protocols
  * UAVtalk
It would be very easy to expand support to new protocols. Very few pieces of data are required from the vehicle to track it.