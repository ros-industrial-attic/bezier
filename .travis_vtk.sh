#!/bin/sh

download_server="http://graissmoto.legtux.org/travis_ci"

sudo apt-get install -qq libvtk5-dev

# Download and install VTK 7.0
wget $download_server/vtk-7.0.0.deb
sudo dpkg -i vtk-7.0.0.deb

