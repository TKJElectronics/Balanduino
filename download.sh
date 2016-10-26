#!/bin/sh -e
# Script to automatically download the project including all submodules
# It then removes all files related to git
# It then zips it, so it is ready to be uploaded to Balanduino.com
# Also used to download the hardware add-on and zip it and calculate SHA-256 and file size
# This is used with the Arduino Boards Manager

url=https://github.com/TKJElectronics/Balanduino.git
hardware_add_on_path=Firmware/hardware/Balanduino/avr

dir="$(cd "$(dirname "$0")" && pwd)"
echo "Working path: $dir"

name=$(echo $(echo $url | rev | cut -d'/' -f 1 | rev) | cut -d'.' -f 1)
echo "\nClone Project: $name\n"
git clone --depth 1 --single-branch -b master --recursive $url || exit 1
cd "$name"

echo "\nRemove git files"
find . -name .git | xargs rm -rf
find . -name .gitmodules | xargs rm -rf
find . -name .gitignore | xargs rm -rf
find . -name .gitattributes | xargs rm -rf

echo "ZIP project directory"
cd "$dir"
zip -rq $name $name

mv "$name/$hardware_add_on_path" "$dir/$name-hardware" # Rename avr directory and move out of hardware folder

echo "ZIP hardware directory"
zip -rq "$name-hardware" "$name-hardware" # Zip the hardware add-on

echo "Remove temporary directories"
rm -rf $name
rm -rf "$name-hardware"

# Calculate SHA-256 and file size of hardware add-on used for "package_tkj_balanduino_index.json"
echo \"checksum\": \"SHA-256:`shasum -a 256 $name-hardware.zip | awk '{print $1}'`\",
echo \"size\": \"`ls -l $name-hardware.zip | awk '{print $5}'`\",
