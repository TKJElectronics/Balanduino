# Balanduino Arduino code
#### Developed by Kristian Lauszus, TKJ Electronics 2013-2014

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
		* Spektrum.ino
		* Tools.ino
	* hardware/
		* Balanduino/
			* avr/
				* bootloaders/
				* libraries/
				* variants/
				* boards.txt
	* libraries/
		* USB\_Host\_Shield\_20/
		* KalmanFilter/

Advanced users can also use the included [Makefile](Makefile) if they prefer.

[Balanduino.hex](Balanduino.hex) contains the latest compiled code.

For more information visit the official website: <http://balanduino.net/>, the forum: <http://forum.balanduino.net/> or feel free to contact us at <support@tkjelectronics.com>.