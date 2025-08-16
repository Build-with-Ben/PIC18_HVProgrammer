# PIC18_HVProgrammer
A simple, Arduino based PIC family High Voltage Programmer using MicroChip ICSP serial programming to help erase, read, or flash programmings to PIC controllers in High Voltage Programming (HVP) mode. 

_This only applies to PIC18(L)F2XK22/4XK22 for now..._

**Helpful Links:**

ICSP Programming Guide for PIC18(L)F2XK22/4XK22 Family

https://ww1.microchip.com/downloads/en/DeviceDoc/41398A.pdf

PIC18(L)F2XK22/4XK22 Datasheet

insert here

**Hardware**: Arduino Uno, PC, target device (PIC18(L)F2XK22/4XK22)

**Problem**: Any PIC controller with both Low Voltage Programming (LVP) and High Voltage Programming (HVP) as programming options has the potential to lock out LVP if configuration bits (fuses) are set to HVP only. Once in HVP mode, LVP only programmers/debuggers will not work. HVP will always work on any PIC thankfully. 

**Goal**: This script aims to provide various operations to re-enable (or disable) the LVP bit within the configuration bits. 


