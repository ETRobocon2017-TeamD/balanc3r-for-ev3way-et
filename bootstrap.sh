#!/bin/sh

# systemctl stop bluetooth
systemctl stop ntp
systemctl stop nmbd
/etc/init.d/avahi-daemon stop

ps lax | awk '{print $3,$6}' | grep -- '-[0-9]\{1,2\}' | awk '{print $1}' | xargs sudo renice 0 -p

nice -n -15 python3 pistol.py
