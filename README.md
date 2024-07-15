# MYACTUATOR_CAN

The objective of this project is to develop a library that allows communication with MYACTUATOR motors using the CAN protocol.

## Setting up 

The library that enables CAN communication using Python is called `python-can`. To install this library, run:
```bash
pip install python-can
```
Then, configure the terminal for CAN communication by running:
```bash
sudo ip link set can0 up type can bitrate 1000000
```
Here, can0 is the name of the real hardware, and bitrate is the bitrate for communication.
