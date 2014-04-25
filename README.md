# Balanduino
#### Developed by TKJ Electronics 2013-2014

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

## Game controllers

To control the robot you can use the following game controllers:

* Normal PS3 Dualshock controller
* PS3 Navigation controller
* PS3 Move controller
* PS4 controller
* Xbox 360 wireless controller
* Wiimote including support for the Motion Plus and Nunchuck extension
* Wii U Pro controller

### Paring

To pair with any of the PS3 controller please see the following section of the USB Host library readme: <https://github.com/felis/USB_Host_Shield_2.0#ps3-library>.

To pair with the PS4 controller send: ```CPP;``` via the serial monitor. Then hold down the Share button and then hold down the PS button without releasing the Share button. The PS4 controller will then start to blink rapidly indicating that it is in paring mode.

To use a Xbox 360 wireless controller you need a special receiver. More information can be seen at the following link: <https://github.com/felis/USB_Host_Shield_2.0#xbox-360-library>.

To pair with the Wiimote send: ```CPW;``` via the serial monitor and then press 1 and 2 at the same time on the Wiimote. On the Wii U Pro controller you should press on the sync button instead.

## Android App

It it also possible to control the robot and adjust the PID values and others variables using the following Android application:

[![Google Play](http://developer.android.com/images/brand/en_generic_rgb_wo_60.png)](http://play.google.com/store/apps/details?id=com.tkjelectronics.balanduino)

Source: <https://github.com/TKJElectronics/BalanduinoAndroidApp>

## Computer applications

You can also use your Windows, Mac or Linux based computer by using the following Processing application: <https://github.com/TKJElectronics/BalanduinoProcessingApp>.

A dedicated Windows application is also available: <https://github.com/TKJElectronics/BalanduinoWindowsApp>.

## RC Transmitter

You can also use a RC transmitter. For now only Spektrum satellite receivers are supported.

You will need the following receiver and cable:

* <http://hobbyking.com/hobbyking/store/__46339__OrangeRx_R100_Spektrum_JR_DSM2_Compatible_Satellite_Receiver.html>
* <http://hobbyking.com/hobbyking/store/__24524__ZYX_S_DSM2_DSMJ_Satellite_Receiver_Cable.html>

After that simply connect the red wire to 5V, black to GND and the yellow one to RX0.

To bind with the satellite receiver send ```BS;``` via the serial monitor and then follow the instructions.

Note that you might need to disconnect the satellite receiver from RX0 if it is already sending data, as it will corrupt your command.

The video below shows a video demonstration of it being used together with some FPV equipment:

<a href="http://www.youtube.com/watch?v=tcFdWlAbc3s" target="_blank"><img src="http://img.youtube.com/vi/tcFdWlAbc3s/3.jpg" alt="Balanduino FPV demonstration" width="240" height="180" border="10" /></a>

# Video presentation

A video for the Kickstarter presentation can be seen below:

<a href="http://www.youtube.com/watch?v=_kQniPbg9zc" target="_blank"><img src="http://img.youtube.com/vi/_kQniPbg9zc/0.jpg" alt="Kickstarter video presentation" width="240" height="180" border="10" /></a>

A video filmed with a GoPro mounted on the robot can be seen below:

<a href="http://www.youtube.com/watch?v=CvFcnb_9anM" target="_blank"><img src="http://img.youtube.com/vi/CvFcnb_9anM/0.jpg" alt="GoPro demonstration" width="240" height="180" border="10" /></a>

# Calibration of the accelerometer

If the robot tilts to one of the sides when you receive the kit, you might need to calibrate the accelerometer.

This is done by simply sending ```AC;``` via the serial monitor. After that simply just follow the instructions.

# Download

To download the entire repository including all submodules run the following command in a terminal:

```bash
curl https://raw.github.com/TKJElectronics/Balanduino/master/download.sh | sh
```

Or simply download the daily updated ZIP-repository from our website: <http://downloads.balanduino.net/github/Balanduino_latest.zip>.

If you want to clone the project then use the recursive command to clone all the submodules as well:

```bash
git clone --recursive https://github.com/TKJElectronics/Balanduino.git
```

</br>

For more information visit the official website: <http://balanduino.net/>, the forum: <http://forum.balanduino.net/> or feel free to contact us at <support@tkjelectronics.com>.