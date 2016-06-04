#!/bin/sh

# Download and install the ensensoSDK
# Note that we should also install uEye but this is not needed for the compilation
wget http://dl.ensenso.de/public/Software/EnsensoSDK/EnsensoSDK-1.3.180-x64.deb -O ensensoSDK.deb
sudo dpkg -i ensensoSDK.deb
rm ensensoSDK.deb

