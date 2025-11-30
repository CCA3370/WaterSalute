# WaterSalute - X-Plane 12 Water Salute Plugin

A native C++ plugin for X-Plane 12 that simulates the traditional water salute ceremony for aircraft. Two fire trucks approach the aircraft and create beautiful water arches as a tribute.

## Features

- **Native X-Plane 12 Plugin**: Built using XPLM 420 SDK
- **Interactive Menu System**: Simple Start/Stop controls in the X-Plane Plugins menu
- **Safety Checks**: Automatically verifies aircraft is on ground with speed < 40 knots
- **Realistic Behavior**:
  - Fire trucks approach from the front of the aircraft
  - Positioning based on actual aircraft wingspan + 40 meters spacing
  - Trucks stop 200 meters ahead and turn to face each other
  - Particle-based water jet effects (controlled by plugin, no .pss files)
- **External 3D Model**: Fire trucks use external OBJ file for customization
- **Smart Menu States**: Start button disabled during operation, Stop enabled only during water spray

## Requirements

- X-Plane 12
- CMake 3.16 or higher
- C++17 compatible compiler:
  - Windows: Visual Studio 2019 or later / MinGW-w64
  - macOS: Xcode 12 or later
  - Linux: GCC 9 or later

## Building

### Linux/macOS

```bash
mkdir build
cd build
cmake ..
make
```

### Windows (Visual Studio)

```bash
mkdir build
cd build
cmake -G "Visual Studio 17 2022" -A x64 ..
cmake --build . --config Release
```

### Windows (MinGW)

```bash
mkdir build
cd build
cmake -G "MinGW Makefiles" ..
mingw32-make
```

## Installation

1. Build the plugin (see above)
2. Copy the `WaterSalute` folder from the build directory to:
   - `X-Plane 12/Resources/plugins/`
3. The folder structure should be:
   ```
   X-Plane 12/
   └── Resources/
       └── plugins/
           └── WaterSalute/
               ├── WaterSalute.xpl (or .dylib on macOS)
               └── resources/
                   ├── firetruck.obj  # X-Plane OBJ8 format
                   └── Panther3.png   # Texture file
   ```

## Usage

1. Start X-Plane 12 and load an aircraft
2. Taxi to a location on the ground (ensure ground speed < 40 knots)
3. Open the menu: **Plugins → Water Salute**
4. Click **Start Water Salute**
5. Watch as two fire trucks approach and create the water arch
6. Click **Stop** when you want to end the ceremony

## How It Works

### Ceremony Sequence

1. **Pre-flight Check**: Verifies aircraft is on ground and moving slowly
2. **Approach Phase**: Two fire trucks spawn 500 meters ahead and drive toward the aircraft
3. **Positioning Phase**: Trucks stop at 200 meters, turn to face each other
4. **Water Spray Phase**: Particle-based water jets create crossing arches over the aircraft
5. **Departure Phase**: When stopped, trucks drive away and disappear

### Technical Details

- Uses X-Plane's instancing API for efficient 3D model rendering
- Terrain probing ensures trucks stay on the ground
- Particle system simulates water physics with gravity
- Flight loop callback for smooth animation updates

## Customization

### Fire Truck Model

Replace `resources/firetruck.obj` with your own 3D model. The model should:
- Be in **X-Plane OBJ8 format** (not Wavefront OBJ)
- Be centered at origin (0, 0, 0) at the bottom
- Face the -Z direction
- Use meters as the unit scale
- Include a water cannon/nozzle at approximately (0, 3.5, 2.0)

### Water Effects

Modify the constants in `src/WaterSalute.cpp`:
- `WATER_JET_HEIGHT`: Maximum height of water arch (default: 25m)
- `NUM_PARTICLES_PER_JET`: Particle density (default: 100)
- `PARTICLE_LIFETIME`: How long particles last (default: 2 seconds)

## Custom Datarefs

The plugin publishes several custom datarefs that can be used by other plugins or tools to control and monitor the fire trucks:

### Truck Control Datarefs

All truck datarefs are float arrays with 2 elements (index 0 = left truck, index 1 = right truck).

| Dataref | Type | Writable | Description |
|---------|------|----------|-------------|
| `watersalute/truck/steering_angle` | float[2] | Yes | Front wheel steering angle in degrees (-45 to 45) |
| `watersalute/truck/wheel_rotation_speed` | float[2] | No | Wheel rotation speed in radians/second (calculated from speed) |
| `watersalute/truck/cannon_pitch` | float[2] | Yes | Water cannon pitch angle in degrees (0 to 90) |
| `watersalute/truck/cannon_yaw` | float[2] | Yes | Water cannon yaw angle in degrees (-180 to 180) |
| `watersalute/truck/speed` | float[2] | No | Current truck speed in meters/second |

### Example Usage

These datarefs can be accessed by other plugins or dataref tools:

```cpp
// Get truck speed dataref
XPLMDataRef speedRef = XPLMFindDataRef("watersalute/truck/speed");

// Read left truck speed (index 0)
float speeds[2];
XPLMGetDatavf(speedRef, speeds, 0, 2);
float leftTruckSpeed = speeds[0];

// Set cannon pitch for right truck (index 1)
XPLMDataRef pitchRef = XPLMFindDataRef("watersalute/truck/cannon_pitch");
float pitchValues[1] = { 60.0f };  // 60 degree pitch
XPLMSetDatavf(pitchRef, pitchValues, 1, 1);  // offset 1, count 1
```

## License

This project is open source. See LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## Acknowledgments

- Laminar Research for X-Plane and the SDK
- The X-Plane development community