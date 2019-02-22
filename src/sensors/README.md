### Sensors Folder

Put all sensor interfaces and implementations here. 


#### Guidlines for sensor APIs
- All initialization work should be done in the constructor
- If possible for the sensor, calibration values should be stored ahead of time and placed in the relevant header files at #defines
- Class constructor should *not* interface with hardware at all!
- Use `init` method to set up hardware
- Use `run` method to enable sensor operation
- Use multiple single-line comments in the relevant cpp file to document each method so it will show up in the IDE

