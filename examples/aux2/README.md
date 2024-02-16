# aux2

This application demonstrates how to retrieve OIS data from AUX2 interface using only AUX driver. It does not need any configuration from UI interface, which can remain undriven.
It gathers accelerometer and/or gyroscope data every ms.

## Specific wiring

### If IMU EVB is used

To allow IMU DB AUX ports to be driven by SmartMotion board MCU, IMU EVB must not be plugged on SmartMotion CN1 but through wires instead :

| **IMU pin**     | **SmartMotion pin** | **IMU EVB pin** |
| :-----:         | :----:              | :-----:         |
| AUX2_CS         | J4.1                | CN3.9           |
| AUX2_SCLK       | J4.2                | CN3.7           |
| AUX2_SDIO       | J4.3                | CN3.5           |
| GND             | J4.7                | CN3.10          |

| **Function**    | **SmartMotion pin** | **IMU EVB pin** |
| :-----:         | :----:              | :-----:         |
| AUX2_SDI        | J4.4                | CN3.8           |
| 5V              | CN1.19              | CN3.4           |

SPI4 to SPI3 conversion is ensured for AUX2 interface by EVB.

### If IMU DB is used

As only SPI3 is supported on AUX2 interface, there is no possibility to execute this example with SmartMotion + IMU daughter board configuration.

## Command interface

This application allows the following command to be sent through UART:

* `a`: to enable/disable accel.
* `g`: to enable/disable gyro.
* `c`: to print current configuration.
* `h`: to print help screen.

## Terminal output

Data are printed on the terminal as follow:

```
<timestamp> us Accel: <acc_x> <acc_y> <acc_z> g Gyro <gyr_x> <gyr_y> <gyr_z> dps
```

With:
* `<timestamp>`: Time in microsecond read from MCU clock when latest INT2 fired.
* `<acc_x|y|z>`: Raw accelerometer value converted in g.
* `<gyr_x|y|z>`: Raw gyroscope value converted in dps.

### Example of output

```
[I] ###
[I] ### Example AUX2
[I] ###
[I]   24685023 us   Accel:   -0.59    -0.56    -0.58 g   Gyro:    0.37     0.37    -0.06 dps
[I]   24686335 us   Accel:   -0.59    -0.56    -0.58 g   Gyro:    0.24    -0.06    -0.37 dps
[I]   24687646 us   Accel:   -0.59    -0.56    -0.58 g   Gyro:    0.31     0.24     0.12 dps
[I]   24688958 us   Accel:   -0.59    -0.56    -0.58 g   Gyro:    0.24     0.24    -0.06 dps
[I]   24690269 us   Accel:   -0.59    -0.56    -0.58 g   Gyro:    0.24     0.12    -0.18 dps
[I]   24691581 us   Accel:   -0.59    -0.56    -0.58 g   Gyro:    0.31    -0.18    -0.49 dps
[I]   24692892 us   Accel:   -0.59    -0.56    -0.58 g   Gyro:    0.49     0.12     0.06 dps
[I]   24694204 us   Accel:   -0.59    -0.56    -0.58 g   Gyro:    0.12     0.06    -0.18 dps
[I]   24695515 us   Accel:   -0.59    -0.56    -0.58 g   Gyro:    0.24     0.12     0.06 dps
[I]   24696827 us   Accel:   -0.59    -0.56    -0.58 g   Gyro:    0.49    -0.06    -0.18 dps

```

