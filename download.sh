#! /bin/bash
# Script to automaticly download a project including all submodules
# It then updates all submodules and removes all files related to git
# It then zips it, so it's ready to upload

url=git@bitbucket.org:Lauszus/balanduino.git

dir="$(cd "$(dirname "$0")" && pwd)"
echo "Working path:" $dir

name=$(echo $(echo $url | rev | cut -d'/' -f 1 | rev) | cut -d'.' -f 1)
echo "\nClone Project:" $name "\n"
git clone --recursive $url

echo "\nUpdate submodules\n"
cd $name
git submodule foreach --recursive git pull origin master

echo "\nRemove git files"
find . -name .git | xargs rm -rf
find . -name .gitmodules | xargs rm -rf
find . -name .gitignore | xargs rm -rf

echo "Rename USB Host Library"
var=$(find . -name USB_Host_Shield_2.0 -type d)
if [ -n "$var" ]
  then
	echo "Path:" $var
	cd "$var"
	cd ../
	mv USB_Host_Shield_2.0 USB_Host_Shield_20
else
	echo "USB Host Library directory not found"
fi

echo "ZIP folder"
cd "$dir"
zip -r -q $name $name

echo "Done!"