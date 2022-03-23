# RAPTOR Position Control Interface

This repository, on a high level, converts *FastDDS* commands to a drone to *MAVSDK* commands which can be sent using a standard telemetry radio or a Xbee. 

## Installation

You will need the following dependencies installed on your system:
- [eProsima FastDDS](https://fast-dds.docs.eprosima.com/en/latest/installation/binaries/binaries_linux.html)
- [MAVSDK](https://mavsdk.mavlink.io/main/en/cpp/guide/installation.html)

In addition, you will also need a telemetry radio (untested) or an Xbee to communicate with the PiHawk 4 flight controller on the drone. 

Once all dependencies are installed, you can build the project using cmake.

## Usage

Once you have built the project, from the root directory, run: 

```bash
./build/apps/position_control/pos_ctrl_interface [path_to_receiver]
```

The path to the receiver may vary from destination to destination:
- For TCP : `tcp://[server_host][:server_port]`
- For UDP : `udp://[bind_host][:bind_port]`
- For Serial : `serial:///path/to/serial/dev[:baudrate (optional)]`

Thus, when using a Xbee, a typical usage would be (the baud rate is optional):
```bash
./build/apps/position_control/pos_ctrl_interface serial:///dev/ttyUSB0
```



      

