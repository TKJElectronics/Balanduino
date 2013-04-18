# Balanduino
#### Developed by Kristian Lauszus and Thomas Jespersen, TKJ Electronics 2013

The code is released under the GNU General Public License.
_________

This is the code for the Balanduino, the Open Source Balancing robot.

# Download

To download the entire repository including all submodules run the following command in a terminal:

```sh
curl https://raw.github.com/TKJElectronics/Balanduino/master/download.sh | sh
```

# Arduino compatible

To use the Balanduino as a normal Arduino board you need to add the hardware add-on to the Arduino IDE.

See the [Hardware Readme](Firmware/hardware/README.md) for more information.

# Hardware

The Balanduino hardware consist of an ATmega644A running at 8MHz.

A MPU-6050 3-axis accelerometer and gyroscope.
The accelerometer and gyroscope readings are combined using the Kalman filter library: <https://github.com/TKJElectronics/KalmanFilter>.

USB Host support is implemented using the MAX3421E and the USB Host library: <https://github.com/felis/USB_Host_Shield_2.0>.

Two VNH5180 motor controller ICs is used to drive the two motors.

More information can be found at the Kickstarter description: <http://www.kickstarter.com/projects/tkjelectronics/balanduino-balancing-robot-kit>.

# Remote control

To control the robot you can use the following game controllers:

* Normal PS3 Dualshock controller
* PS3 Navigation controller
* PS3 Move controller
* Xbox 360 wireless controller
* Wiimote including support for the Motion Plus and Nunchuck extension
* Wii U Pro controller

It it also possible to control the robot and adjust the PID values and others variables using the following Android application:

[![Google Play](http://developer.android.com/images/brand/en_generic_rgb_wo_60.png)](http://play.google.com/store/apps/details?id=com.tkjelectronics.balanduino)

Source: <https://github.com/TKJElectronics/BalanduinoAndroidApp>

Or by using the Processing application: <https://github.com/TKJElectronics/BalanduinoProcessingApp>.

# Video presentation

A video for the Kickstarter presentation of the robot can be found here: <http://www.youtube.com/watch?v=_kQniPbg9zc>.

A video filmed with a GoPro mounted on the robot can be seen here: <http://www.youtube.com/watch?v=CvFcnb_9anM>.

For more information send us an email at <mail@tkjelectronics.com>.