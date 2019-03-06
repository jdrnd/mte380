## MTE 380 Code

#### Getting Started
We're using PlatformIO to manage our code and board dependancies. Here's the getting started guide: https://docs.platformio.org/en/latest/ide/vscode.html#installation

Make sure you install for VSCode instead of Atom on *Windows*. The PlatformIO ide provides the build, flash, and serial monitor in the bottom toolbar. The board settings are in the `platformio.ini` file.

#### General git workflow
Start a new branch, commit to that branch, rebase onto master and open a pull request, get it approved, rebase again and merge the PR.

#### Development
Current options are kept both in the `platformio.ini` file and in the `common.h` file. These options are flags that determine what funcionality we run. For example, if RUN_IMU is not defined we will not initialize or use the IMU.

Debugging: The XBEE on the robot in connected to Serial3 and is to be used for logging sensor readings. The XBEE serial port is passed via the PLOTTER_SERIAL macro which is set in platformio.ini. The main USB serial is for debug purposes. Use the DEBUG_PRINT (defined in common.h) macro to print instead of Serial.println. 

Using the XBEE for Remote Debugging: Sensor values will print over the XBEE radio link. You should have another XBEE module hooked up to your computer via usb. You can then open it as a generic serial port, which will work with Putty, SerialPlot, or the Arduino Plotter listener. If the serial port does not appear and the module appears to be on, you may need to download the XBEE driver software: https://www.digi.com/products/iot-platform/xctu

#### Task Layout
Elements of the robot's functionality are broken down into tasks, which are run via a cooperative, *non-preemptive* scheduler (https://github.com/arkhipenko/TaskScheduler). The tasks are setup in `main.cpp`, and each task has it's own `.cpp/.h` files in the `src/task` folder. Note that tasks can enable/disable themselves and others, as well as reconfigure parameters such as their period. Tasks run roughly in the order that they are declared in `main.cpp` except for when they have different periods.

Interrupts should be avoided except for setting flags. The exception here is for the motor interrupts, which set the current motor speed.

#### Utilities/Logging Tools
SerialPlot: to plot recieved sensor readings in real time: https://hackaday.io/project/5334-serialplot-realtime-plotting-software
Arduino Plotter: we use this library on the arduino side in order to send the sensor values in a nice format over serial. In order to plot them on a graph you will need the plotter listener running. See https://github.com/devinaconley/arduino-plotter
Putty: you can open a com/serial port with Putty to see what is going on
