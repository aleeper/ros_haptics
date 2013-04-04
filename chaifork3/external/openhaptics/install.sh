#!/bin/sh
unzip sensable.zip
cd sensable
sudo ./install.sh
cd ..
sudo cp 80-firewire.rules /etc/udev/rules.d/
sudo cp /usr/lib64/* /usr/lib/
sudo ldconfig

