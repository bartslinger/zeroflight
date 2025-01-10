# Introduction

This project is an experimental flight controller firmware. It is not a replacement for autopilot firmwares such as
Ardupilot, PX4 or INAV.

The primary use-case at the moment is a fixed-wing airplane equipped with a SpeedyBee F405 Wing (Mini) flight controller
with an ExpressLRS radio receiver. If you want to use it on a quadcopter or a different type of vehicle, you may need to
update the control algorithm.

The functionality is currently limited to stabilization. It does not use the GPS or compass. The three available flight
modes are:

- Stabilized
- Acro
- Manual