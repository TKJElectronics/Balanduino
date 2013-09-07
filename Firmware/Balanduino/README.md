# Balanduino Arduino code
#### Developed by Kristian Lauszus, TKJ Electronics 2013

The code is released under the GNU General Public License.
_________

This is the Arduino code for the Balanduino balancing robot.

To compile the sketch you will need the newest version of the [Arduino IDE](http://arduino.cc/en/Main/Software).

Then open the sketch by clicking on [Balanduino.ino](Balanduino.ino).

The final structure should look like this:

* Arduino/
	* Balanduino/
		* Balanduino.ino
		* Balanduino.h
		* Bluetooth.ino
		* EEPROM.ino
		* EEPROMAnything.h
		* I2C.ino
		* Motor.ino
		* Tools.ino
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

Advanced users can also use the included [Makefile](Makefile) if they prefer.

For more information send us an email at <mail@tkjelectronics.com>.