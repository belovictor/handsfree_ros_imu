#!/bin/bash

sudo cp ./scripts/usb_rules/handsfree-serial.rules /etc/udev/rules.d/
sudo cp ./scripts/usb_rules/56-orbbec-usb.rules /etc/udev/rules.d/56-orbbec-usb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add
sudo service udev reload
sudo service udev restart

