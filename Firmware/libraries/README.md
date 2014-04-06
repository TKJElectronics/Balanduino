# Balanduino libraries
#### Developed by Kristian Lauszus, TKJ Electronics 2013-2014

The code is released under the GNU General Public License.
_________

These are the needed libraries in order to compile the code.

To use it, simply download the libraries and moved them into your [libraries folder](http://arduino.cc/en/Guide/Libraries).

__NB:__ The libraries folders are not allowed to have any special characters, so rename the libraries to:

* USB\_Host\_Shield\_20
* KalmanFilter

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
				* platform.txt
	* libraries/
		* USB\_Host\_Shield\_20/
		* KalmanFilter/

For more information visit the official website: <http://balanduino.net/>, the forum: <http://forum.balanduino.net/> or feel free to contact us at <support@tkjelectronics.com>.