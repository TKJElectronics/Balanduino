#! /bin/sh -e
# Script to automatically download the project including all submodules
# It then updates all submodules and removes all files related to git
# It then zips it, so it is ready to be uploaded to Balanduino.com

url=https://github.com/TKJElectronics/Balanduino.git

dir="$(cd "$(dirname "$0")" && pwd)"
echo "Working path:" $dir

name=$(echo $(echo $url | rev | cut -d'/' -f 1 | rev) | cut -d'.' -f 1)
echo "\nClone Project:" $name "\n"
git clone --recursive $url || exit 1
cd "$name"

echo "\nRemove git files"
find . -name .git | xargs rm -rf
find . -name .gitmodules | xargs rm -rf
find . -name .gitignore | xargs rm -rf

echo "ZIP directory"
cd "$dir"
zip -rq $name $name

echo "Remove temporary directory"
rm -rf $name

echo "Done!"