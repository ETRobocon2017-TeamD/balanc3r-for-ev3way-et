#!/bin/sh

# systemctl stop bluetooth
systemctl stop ntp
systemctl stop nmbd
/etc/init.d/avahi-daemon stop

ps lax | grep -- [-]20 | awk '{print $3}' | xargs sudo renice 0 -p

nice -n -13 python3 pistol.py
