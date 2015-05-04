#! /bin/sh -e
# Script to automatically download the hardware add-on and zip it and calculate SHA-256 and file size.
# This is used with the Arduino Boards Manager.

url=https://github.com/TKJElectronics/Balanduino.git
hardware_path=Firmware/hardware

dir="$(cd "$(dirname "$0")" && pwd)"
echo "Working path: $dir"

name=$(echo $(echo $url | rev | cut -d'/' -f 1 | rev) | cut -d'.' -f 1)
echo "\nClone Project: $name\n"
git clone $url || exit 1

cd "$name/$hardware_path/$name"

mv avr "$dir"

cd "$dir"
rm -rf $name
mv avr $name

echo "ZIP directory"
zip -rq $name $name

echo "Remove temporary directory"
rm -rf $name

echo \"checksum\": \"SHA-256:`shasum -a 256 $name.zip | awk '{print $1}'`\",
echo \"size\": \"`ls -l $name.zip | awk '{print $5}'`\",