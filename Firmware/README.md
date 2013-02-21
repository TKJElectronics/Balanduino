# Balanduino firmware
#### Developed by Kristian Lauszus and Thomas Jespersen, TKJ Electronics 2013

The code is released under the GNU General Public License.
_________

This is the code for the Balanduino balancing robot.

You should put all these directories and files inside your Arduino sketchbook folder. The final structure should look like this:

* Arduino/
	* Balanduino/
		* Balanduino.ino
		* Balanduino.h
		* Bluetooth.ino
		* I2C.ino
		* Motor.ino
		* PID.ino	
	* hardware/
		* Balanduino/
			* avr/
			* bootloaders/
			* cores/
			* variants/
			* boards.txt
	* libraries/
		* USB\_Host\_Shield\_20/
		* KalmanFilter/	

# Skal ændres

It's a ported version of the code for the mbed board which I original used. The original code can be found at the following link: [https://github.com/TKJElectronics/BalancingRobot](https://github.com/TKJElectronics/BalancingRobot).

The code will work for all boards that features an ATmega328p (Duemilanove, Uno, Pro, Pro Mini etc.) - it's not directly pin compatible with the larger Arduinos (Mega, Mega 2560 etc.) as I use the port registers to save processing resources - see [http://www.arduino.cc/en/Reference/PortManipulation](http://www.arduino.cc/en/Reference/PortManipulation). But just take a look at the pinMapping pages for comparison: [http://arduino.cc/en/Hacking/PinMapping168](http://arduino.cc/en/Hacking/PinMapping168) and [http://arduino.cc/en/Hacking/PinMapping2560](http://arduino.cc/en/Hacking/PinMapping2560) and figure the pins out yourself - for instance OC1A and OC1B are not located on pin 9 and 10, but at pin 11 and 12 on the Arduino Mega.

I have used the registers to set up 20kHz PWM, Phase and Frequency Correct on pin 9 (OC1A) & pin 10 (OC1B) with ICR1 as TOP using Timer1 - see the [datasheet](http://www.atmel.com/Images/doc8025.pdf) page 128-135.

I use a 6DOF IMU from Sparkfun: [http://www.sparkfun.com/products/10010](http://www.sparkfun.com/products/10010), though I only use one of the gyro axis, but any IMU can be used. For instance the very popular MPU-6050, see this [example code](https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter/blob/master/IMU6DOF/MPU6050/MPU6050.ino).

For more info about calculating the pitch see my post at the Arduino forum: [http://arduino.cc/forum/index.php/topic,58048.0.html](http://arduino.cc/forum/index.php/topic,58048.0.html).

For more information about the Kalman filter see my blog post: [http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/](http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/) and the source code: [https://github.com/TKJElectronics/KalmanFilter](https://github.com/TKJElectronics/KalmanFilter).

To steer the robot, I use a [USB Host Shield](http://www.circuitsathome.com/products-page/arduino-shields/usb-host-shield-2-0-for-arduino/) together with my SPP Bluetooth Library for Arduino: [https://github.com/felis/USB_Host_Shield_2.0/blob/master/SPP.cpp](https://github.com/felis/USB_Host_Shield_2.0/blob/master/SPP.cpp).
More information can be found at the blog psot: [http://blog.tkjelectronics.dk/2012/07/rfcommspp-library-for-arduino/](http://blog.tkjelectronics.dk/2012/07/rfcommspp-library-for-arduino/).

You can either use an Android app I wrote: [https://github.com/TKJElectronics/BalanduinoAndroidApp](https://github.com/TKJElectronics/BalanduinoAndroidApp) or the [Processing Application](https://github.com/TKJElectronics/BalancingRobotArduino/tree/master/ProcessingApp) to control the robot.

For information about the hardware, see the wiki: [https://github.com/TKJElectronics/BalancingRobot/wiki/Hardware](https://github.com/TKJElectronics/BalancingRobot/wiki/Hardware).

Also check out the youtube video of it in action: [http://www.youtube.com/watch?v=N28C_JqVhGU](http://www.youtube.com/watch?v=N28C_JqVhGU) - this is actually the mbed version, but they behave the same way.

For more information see my blog post at [http://blog.tkjelectronics.dk/2012/03/the-balancing-robot/](http://blog.tkjelectronics.dk/2012/03/the-balancing-robot/) or send me an email at <a href="mailto:kristianl@tkjelectronics.dk?Subject=BalanduinoAndroidApp">kristianl@tkjelectronics.dk</a>.