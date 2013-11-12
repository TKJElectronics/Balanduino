# Balanduino hardware add-on
#### Developed by Kristian Lauszus, TKJ Electronics 2013

The code is released under the GNU General Public License.
_________

This is the Balanduino hardware add-on for the Arduino IDE.

To use it, simply create a folder named hardware inside your [sketchbook](http://arduino.cc/en/Guide/Environment#sketchbook) directory.

Now move the Balanduino directory inside that folder.

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
				* libraries/
				* variants/
				* boards.txt
				* platform.txt
	* libraries/
		* USB\_Host\_Shield\_20/
		* KalmanFilter/

For more information see the following site: <http://www.arduino.cc/en/Guide/Environment#thirdpartyhardware> or send us an email at <mail@tkjelectronics.com>.