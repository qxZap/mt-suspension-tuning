# Suspension Visuals

Brake disc, caliper, spacer, knuckle, coilover (rigid eyes, telescoping damper, stretching spring), lower arm and drive shaft (stub, CV joints with boots) at every wheel of the nearest vehicles, following the live suspension and spinning with the wheel. Eight Paint screen entries (Brake Caliper, Brake Disc, Suspension, Dampers, Wheel Spacer, Lug Nuts, Knuckle & Arms, Drive Shaft). Tuning > Brakes & Suspension: per axle, brake position, spacer length and size, upper/lower mount offsets, coilover top and size, lower arm pivot and mount spacing, shaft joint position; copy/paste between axles.

Needs the pak `qxZap_SuspensionVisuals_P` (the meshes under `/Game/SuspensionVisuals/` and the Big Brake Kit rows, Brake Pads, in the DataTableFramework reserved slot Slot01). Without it the pieces are plain shapes.

Settings: `Binaries\Win64\plugins\suspension_visuals.ini`, `[SuspensionVisuals]` Range (60 m), MaxVehicles (8), BrakeInset (0.25). The per-car setup and paint are stored on the car (packed colour entries QXS<axle><A-F>, QX_*) and in this ini, which restores them when the game drops entries it does not know.

## Install

Needs [Plugin Loader](https://github.com/qxZap/mt-plugin-loader) (`version.dll`) in
`MotorTown\Binaries\Win64\`. Put `mt_plugin_SuspensionVisuals.dll` in `MotorTown\Binaries\Win64\plugins\`.

## Build

`build.bat` (MSVC 2022, x64) writes `out\mt_plugin_SuspensionVisuals.dll`.
