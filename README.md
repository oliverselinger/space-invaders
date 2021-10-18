# space-invaders

Intel 8080 emulator written from scratch in ZIG, to run Space Invaders.
SDL is used as video interface.

![screenshot](https://github.com/oliverselinger/space-invaders/blob/main/screenshot.png?raw=true)

* No audio support
* No multiplayer support

## Dependencies

* ZIG 0.8.1
* SDL2

## Build and Run and Release

`zig build run`

`zig build release`

## Play

| Key | Action |
|-----|--------|
|c    |insert coin|
|p    |1 player - start game|
|left |left|
|right|right|
|s    |fire|

Thanks to:
- [emulator101](http://www.emulator101.com)
- [bluishcoder](https://bluishcoder.co.nz/js8080) for debugging
- [Lemonboy](https://github.com/LemonBoy/Space-Invaders-Emulator)
