# IoTwx AeroNode

Arduino implementation for IoTwx AERO node.



The basic operation requires the digital compass heading to be used for wind direction, and the infrared sensor as a counter for the windspeed with conversion to MPH.  If you would like kmPH, you will need to adjust in your software or updated the `.ino `file.  Future implementations will likely parameterize that in the configuration file.



This implementation depends on two sensors:

* [Adafruit LSM303AGR_Mag](https://www.arduino.cc/reference//en/libraries/adafruit-lsm303agr-mag/), the specific sensor used is the Qwiic LSM303d that can be found [here](https://www.mouser.com/ProductDetail/Adafruit/4413?qs=sGAEpiMZZMu3sxpa5v1qrs7aFKzpKeg1iy7itN8NqEg%3D).  That sensor requires an inexpensive [Qwicc to Grove adapter cable](https://www.mouser.com/ProductDetail/Adafruit/4424?qs=CUBnOrq4ZJyUa%252BR7VYX6Vw%3D%3D) to work on IoTwx.
* [SeeedStudio Grove Infrared Reflective Sensor](https://wiki.seeedstudio.com/Grove-Infrared_Reflective_Sensor/)



The implementation requires 2 cores on the microcontroller and has been tested on the following microcontroller:

* [m5Stack Atom Lite](https://m5stack-store.myshopify.com/collections/m5-atom/products/atom-lite-esp32-development-kit)



You will want to adjust the config file to operate at 160Mhz or higher  by setting the `iotwx_max_frequency` parameter in [`data/config.json`](./data/config.json):

```json
    "iotwx_max_frequency":"240"
```

## Hardware requirements

The code expects at least one of the sensors to be connected.  If you only have the IR sensor connected, you can connected it directly to 
the Grove port.  If you choose to install both sensors, make certain the code is adjusted accordingly.  On the AtomLite, the Grove pins
are 26 and 32, thus the default in code is set with the interrupt pin for the digital infrared sensor on pin 25 (the GPIO pin on the right side
for digital input).  If you choose not to use the GPIO and instead the Grove connector, change the interrupt pin accordingly (to pin 26):

```c
...

int             publish_interval; 
int             max_frequency       = 80;    
const byte      interruptPin        = 25;  // <-=-=! CHANGE THE INTERRUPT PIN HERE, DEFAULT = 25 FOR ATOM LITE GPIO
volatile int    interruptCounter    = 0;

...
```

Please see the [STL files](https://github.com/NCAR/iotwx-manual/tree/main/build/stl) for printing the pin harness adapter.
