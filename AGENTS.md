# AGENTS.md -- Suspension Visuals

A native Motor Town plugin: one DLL loaded by
[Plugin Loader](https://github.com/qxZap/mt-plugin-loader). Source is `plugins/suspension_visuals.cpp`.
`include/` is a copy of the loader's headers; refresh it from mt-plugin-loader when the
api version changes. The source of truth is `D:\MTLiveMap\mt_loader\plugins\suspension_visuals.cpp`; copy changes here.

## Build and test

`build.bat` (MSVC 2022 x64) writes `out\`. Install to `MotorTown\Binaries\Win64\plugins\`
with the game closed. There is no test suite: the log `plugins\mt_plugin_<Name>.log` in that
folder is the check, so log what you need to see in game.

## Rules

- The loader owns the ProcessEvent hook and engine discovery. Use `include/mt_plugin_api.h`,
  and never hook a vtable from a plugin.
- `process_event` only on the game thread (`run_on_game_thread`).
- Find fields by reflection (`api->field_offset`) and code by signature (`find_sig`), never
  by address. A missing field or signature logs a line and leaves the game vanilla.
- Hardcoded offsets are the exception and need a `shortcut:` comment naming the limit.
- Mappings come from the loader: `[Mappings] Export=1` in `mt_loader.ini`. After a game
  update recheck every signature and hardcoded offset.
- A plugin rewriting game data affects the machine running the game: single player or the
  host. On someone else's server that server's rules apply.
- Settings live in `Binaries\Win64\plugins\<plugin>.ini`, through the loader's ini functions (`iniFile` moves an old file over). `[General] Disabled=1` makes `PluginInit` start nothing. After
  renaming one, move the old file on start so players keep their settings.

## This plugin

Brake disc, caliper, spacer, knuckle, coilover (rigid eyes, telescoping damper, stretching spring), lower arm and drive shaft (stub, CV joints with boots) at every wheel of the nearest vehicles, following the live suspension and spinning with the wheel. Eight Paint screen entries (Brake Caliper, Brake Disc, Suspension, Dampers, Wheel Spacer, Lug Nuts, Knuckle & Arms, Drive Shaft). Tuning > Brakes & Suspension: per axle, brake position, spacer length and size, upper/lower mount offsets, coilover top and size, lower arm pivot and mount spacing, shaft joint position; copy/paste between axles.

- The Paint screen greys out an entry unless a mesh on the car has a material slot of that name: the plugin adds QX_* to `MTVehicle.ColorSlots` and `Customization.BodyColors` (names made with Conv_StringToName) and renames its meshes' slots in memory.
- The game drops unknown colour entries on load and when it rebuilds the list in the garage; the ini copy puts them back.
- Mounts are placed from the hub's straight-ahead position: the steering axis passes through them, so they never move when the wheel steers.
- Read slot order and piece dimensions off the assets (Arini's generator scripts), never guess them.
- Dirt: each corner's materials copy every Dirt* parameter from its own wheel's material.
- A caught fault logs its step, address and module.
