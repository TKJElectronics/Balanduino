# You should only have to change these two values
PORT = /dev/tty.usbserial-DN0031EV
ARD_REV = 105
ARD_HOME = /Applications/Arduino1.0.5.app/Contents/Resources/Java

# Uncomment to activate serial debugging
#EXTRA_FLAGS += -D DEBUG_USB_HOST

# Leave these alone
THIRD_PARTY_HARDWARE = ../hardware/Balanduino
BOARD = balanduino
EXTRA_FLAGS += -D ARDUINO_AVR_BALANDUINO
MON_SPEED = 115200

LIB_DIRS += ../libraries/USB_Host_Shield_2.0
LIB_DIRS += ../libraries/KalmanFilter
LIB_DIRS += $(ARD_HOME)/libraries/Wire
LIB_DIRS += $(ARD_HOME)/libraries/Wire/utility

include Arduino_Makefile_master/_Makefile.master