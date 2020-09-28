# A32NX Fly-by-Wire System
Fly-by-wire system implementation for the [flybywire/a32nx](https://github.com/flybywiresim/a32nx) as WASM gauge.

## Introduction
This project seeks to model the fly-by-wire system in the A320 as accurately as possible given the current access to various data sources.

#### Current FBW Features
- Pitch attitude protection
- Load factor limitation
- High speed protection
- High angle of attack protection
- Bank angle protection
- Load factor demand
- Flare mode

#### Upcoming Features
- Autotrim
- Smoothness improvements 
- Alternate and direct laws
- Better ground to flight transition
- Better angle of attack protection
- Manage available protections/law based on the state of ELAC/SEC/FAC and other systems

## Installation

To install the FBW system:
1. Compile the project.
2. Move the generated `FBW.wasm` file to the `A32NX\SimObjects\AirPlanes\Asobo_A320_NEO\panel` folder.
3. Copy the `A32NX` folder into the [flybywire/a32nx](https://github.com/flybywiresim/a32nx) project.
4. Run the `build.py` script in the `A32NX` folder in the [flybywire/a32nx](https://github.com/flybywiresim/a32nx) project.
5. Follow the installation instructions for the [flybywire/a32nx](https://github.com/flybywiresim/a32nx) project.

## Compiling

You can compile the gauge by installing Visual Studio 2019 and the Microsoft Flight Simulator 2020 SDK.

Once installed, use Visual Studio 2019 to open the `FBW.sln` file, and from there it can be compiled.

The compilation will create a file in `MSFS\Debug\FBW.wasm` or `MSFS\Release\FBW.wasm`

#### Compilation Without Visual Studio

Compilation without Visual Studio is not supported at this time.

## Known issues

#### The FBW system is jerky/unsmooth and doesn't keep me smoothly within the flight envelope

While the fundamental logic of the FBW system is sound, it relies on tuned PID controllers to function effectively.
As time progresses, more work will be done to tune the PID controllers so that the input will become smoother and smoother.

#### Alpha floor does not work

This action is baked into the default FBW flight model. By disabling it and replacing it with our own, we lose that protection.
This will need to be fixed upstream in the main A32NX project or A32NX systems project.

#### High angle of attack protection does not maintain best climb rate/angle

High angle of attack protection is implemented using data from the FCOM.
Unfortunately, this does not seem to match up with the flight model in the sim (the plane's climb angle is not maximum at the 1G stall speed).
Work will need to be done to figure out best climb angle/rate from the current flight model at different parameters and then that can be used.
Alternatively, the flight model can be tuned.

#### The ELAC/SEC/FAC buttons are inoperative

The ELAC/SEC/FAC buttons are tied to the default FBW state. By disabling it and replacing it with our own, they show as disabled.
This will need to be fixed upstream in the main A32NX project or A32NX systems project.

#### The thrust levers cannot be moved without disconnecting autothrottle

This action is baked into the default FBW flight model. By disabling it and replacing it with our own, we lose that protection.
This will need to be fixed upstream in the main A32NX project or A32NX systems project.