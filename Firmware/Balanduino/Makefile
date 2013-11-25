# You should only have to change this value:
PORT = /dev/tty.usbserial-DNWYFIDM

# You should only set the path to your Arduino application if it is not the default one:
# The default for Linux and Solaris is:
#ARD_HOME = /opt/Arduino
# The default for Mac is:
#ARD_HOME = /Applications/Arduino.app

# Uncomment to activate serial debugging for the USB Host library:
#EXTRA_FLAGS += -D DEBUG_USB_HOST

# Leave these alone
THIRD_PARTY_HARDWARE = ../hardware/Balanduino/avr
BOARD = balanduino

LIB_DIRS += ../libraries/USB_Host_Shield_2.0
LIB_DIRS += ../libraries/KalmanFilter
LIB_DIRS += ../libraries/Arduino-PID-Library/PID_v1
LIB_DIRS += $(ARD_HOME)/hardware/arduino/avr/libraries/Wire
LIB_DIRS += $(ARD_HOME)/hardware/arduino/avr/libraries/Wire/utility

include Arduino_Makefile_master/_Makefile.master