## Members
Stephen Welch, Computer Engineering Student (2024)
stephenwelch@vt.edu

## Mentor
MENTOR NAME HERE

## Current Status
IN PROGRESS

## Project Overview

3D-printed biped for <$1000.

## Educational Value Added

- Learning how to CAD complex assemblies
- Embedded controls
- High-level controls

## Tasks

<!-- Your Text Here. You may work with your mentor on this later when they are assigned -->

## Design Decisions

- Tethered operation to reduce cost
- Battery, RPi, Power electronics to be added later
- Based on Tello Leg: [Video 1](https://www.youtube.com/watch?v=62lo04Up2vc) [Video 2](https://www.youtube.com/watch?v=mn8tCtYHzHI&t=1s) [Paper](https://arxiv.org/abs/2203.00644)
- Avoid gear/belt transmissions where possible
    - Large losses to friction, etc. esp. at this scale
    - Hard to get right, esp. with 3D prints
- Minimize leg inertia

## Design Misc

Current Risks:
Torque output, esp. at hip linkage
Knee linkage

## Steps for Documenting Your Design Process

<!-- Your Text Here. You may work with your mentor on this later when they are assigned -->

## BOM + Component Cost
|Component|Quantity (Per-leg)|Quantity (Unit)|Unit Price|Total|
|---------|------------------|---------------|----------|-----|
|[Elmo](https://www.amazon.com/Sesame-Street-Tickliest-Laughing-Toddlers/dp/B08TJ44LR9/ref=sr_1_2?crid=1QN8HDIISQ9N2&keywords=tickle+me+elmo&qid=1697677883&s=toys-and-games&sprefix=tickle+me+elmo%2Ctoys-and-games%2C90&sr=1-2)|N/A|1|$50|$50|
|[RM-C610 Brushless Motor Controller](https://store.dji.com/product/rm-c610-brushless-dc-motor-speed-control)|3|N/A|$39|$117|
|[RM-M2006-C610 Brushless Motor](https://store.dji.com/product/rm-m2006-p36-brushless-motor)|3|N/A|$55|$165|
|[6656K118](https://www.mcmaster.com/products/bearings/ball-bearings~/ultra-thin-ball-bearings-6/)|2|N/A|$15.65|$31.30|
|[Skate Bearings](https://www.amazon.com/gp/product/B08YN6WQXR/ref=sw_img_1?smid=AED51UXSJ2INU&psc=1)|2|20|N/A|N/A|
|[Teensy 4.1](https://www.amazon.com/PJRC-Teensy-4-1-Without-Pins/dp/B088D3FWR7)|N/A|1|$31.99|$31.99|

## Timeline

<!-- Your Text Here. You may work with your mentor on this later when they are assigned -->

## Useful Links

[CAD](https://cad.onshape.com/documents/4743a97557c0a80d1585b0a7/w/82680a303504770a7b3fa862)

## Log

<!-- Your Text Here. You may work with your mentor on this later when they are assigned -->
