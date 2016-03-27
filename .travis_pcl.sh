#!/bin/sh

download_server="http://graissmoto.legtux.org/travis_ci"

# Dependencies
sudo add-apt-repository -y ppa:v-launchpad-jochen-sprickerhof-de/pcl > /dev/null
sudo apt-get update > /dev/null
sudo apt-get -qq install -y libeigen3-dev libflann-dev libboost-all-dev libopenni-dev libopenni2-dev

# Download and extract pcl 1.8.0
wget $download_server/pcl-1.8.0.zip
unzip pcl-1.8.0.zip > /dev/null
rm pcl-1.8.0.zip
sudo rsync -a pcl-1.8.0/* /usr/local/
sudo rm pcl-1.8.0 -rf

