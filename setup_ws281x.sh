#!/bin/bash
cd ../rpi_ws281x/rp1_ws281x_pwm
sudo insmod ./rp1_ws281x_pwm.ko pwm_channel=0
sudo dtoverlay -d . rp1_ws281x_pwm
sudo pinctrl set 12 a0 pn