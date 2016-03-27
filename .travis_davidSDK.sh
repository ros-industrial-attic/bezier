#!/bin/sh

# Clone, configure, compile and install the davidSDK
dir=`pwd`
cd ..
mkdir -p davidSDK && cd davidSDK
git clone https://github.com/InstitutMaupertuis/davidSDK.git src
mkdir -p build && cd build
cmake ../src > /dev/null
echo "davidSDK CMake configuration successful"
make -j2 > /dev/null
echo "davidSDK compilation successful"
sudo make -j2 install > /dev/null
echo "davidSDK install successful"
cd $dir

