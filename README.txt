WaterSalute - X-Plane 12 Water Salute Plugin
=============================================

A native C++ plugin for X-Plane 12 that simulates the traditional water salute
ceremony for aircraft. Two fire trucks approach the aircraft and create
beautiful water arches as a tribute.

INSTALLATION
------------

1. Copy the entire "WaterSalute" folder to:
   X-Plane 12/Resources/plugins/

2. The folder structure should be:
   X-Plane 12/
   └── Resources/
       └── plugins/
           └── WaterSalute/
               ├── resources/
               │   ├── firetruck.obj
               │   └── Panther3.png
               ├── lin_x64/
               │   └── WaterSalute.xpl  (Linux)
               ├── win_x64/
               │   └── WaterSalute.xpl  (Windows)
               ├── mac_x64/
               │   └── WaterSalute.xpl  (macOS)
               ├── README.txt
               └── LICENSE.txt

USAGE
-----

1. Start X-Plane 12 and load an aircraft
2. Taxi to a location on the ground (ensure ground speed < 40 knots)
3. Open the menu: Plugins → Water Salute
4. Click "Start Water Salute"
5. Watch as two fire trucks approach and create the water arch
6. Click "Stop" when you want to end the ceremony

FEATURES
--------

- Native X-Plane 12 Plugin built using XPLM 420 SDK
- Interactive Menu System with simple Start/Stop controls
- Safety Checks: Automatically verifies aircraft is on ground with speed < 40 knots
- Realistic Behavior:
  * Fire trucks approach from the front of the aircraft
  * Positioning based on actual aircraft wingspan + 40 meters spacing
  * Trucks stop 200 meters ahead and turn to face each other
  * Particle-based water jet effects

LICENSE
-------

This project is licensed under the GNU General Public License v3.
See LICENSE.txt for full details.

For more information, visit the project repository.
