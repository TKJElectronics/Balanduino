# Balanduino
#### Developed by Kristian Lauszus, TKJ Electronics 2013

The code is released under the GNU General Public License.
_________

This is the code for the Balanduino, the Open Source Balancing robot.

# Arduino compatible

To use the Balanduino as a normal Arduino board you need to add the hardware add-on to the Arduino IDE.

See the [Hardware Readme](Firmware/hardware/README.md) for more information.

# Hardware

The Balanduino hardware consist of an ATmega1284P running at 10MHz.

A MPU-6050 3-axis accelerometer and gyroscope.
The accelerometer and gyroscope readings are combined using a Kalman filter library. See my blog post: <http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/> and source <https://github.com/TKJElectronics/KalmanFilter> for more information.

USB Host support is implemented using the MAX3421E and the USB Host library: <https://github.com/felis/USB_Host_Shield_2.0>.

Two VNH5180 motor controller ICs is used to drive the two motors.

More information can be found at the Wiki article: <http://wiki.balanduino.net/Overview>.

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

You can also use your Windows, Mac or Linux based computer by using the following Processing application: <https://github.com/TKJElectronics/BalanduinoProcessingApp>.

A dedicated Windows application is also available: <https://github.com/TKJElectronics/BalanduinoWindowsApp>.

# Video presentation

A video for the Kickstarter presentation of the robot can be found here: <http://www.youtube.com/watch?v=_kQniPbg9zc>.

A video filmed with a GoPro mounted on the robot can be seen here: <http://www.youtube.com/watch?v=CvFcnb_9anM>.

# Download

To download the entire repository including all submodules run the following command in a terminal:

```bash
curl https://raw.github.com/TKJElectronics/Balanduino/master/download.sh | sh
```

To clone the project use the recursive command to clone all the submodules as well:

```bash
git clone --recursive https://github.com/TKJElectronics/Balanduino.git
```

To update all submodules run the following command:

```bash
git submodule foreach --recursive git pull origin master
```

For more information send us an email at <mail@tkjelectronics.com>.