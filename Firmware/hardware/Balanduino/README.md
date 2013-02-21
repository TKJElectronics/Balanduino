# Balanduino hardware add-on
#### Developed by Kristian Lauszus and Thomas Jespersen, TKJ Electronics 2013

The code is released under the GNU General Public License.
_________

This is the Balanduino hardware add-on for the Arduino IDE.

To use it, simply create a folder named hardware inside your sketchbook directory.

Now move the Balanduino directory inside that folder. The final structure should look like this:

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

For more information see the following site: [http://www.arduino.cc/en/Guide/Environment#thirdpartyhardware](http://www.arduino.cc/en/Guide/Environment#thirdpartyhardware)
or send me an email at <kristianl@tkjelectronics.dk>.