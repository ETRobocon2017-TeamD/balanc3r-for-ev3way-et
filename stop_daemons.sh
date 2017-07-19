#!/bin/sh

# systemctl stop bluetooth
systemctl stop ntp
systemctl stop nmbd
/etc/init.d/avahi-daemon stop
