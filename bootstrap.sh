#!/bin/sh

usage_exit() {
        echo "Usage: $0 [-r||-l] [-d]" 1>&2
        exit 1
}

while getopts rld OPT
do
    case $OPT in
        r)  FLAG_R=1
            ;;
        l)  FLAG_L=1
            ;;
        d)  FLAG_DAEMON=1
            ;;
        \?) usage_exit
            ;;
    esac
done

if [ "$FLAG_DAEMON" ]; then
    echo "Stop daemons"
    systemctl stop ntp
    systemctl stop nmbd
    /etc/init.d/avahi-daemon stop
fi

ps lax | grep -- [-]20 | awk '{print $3}' | xargs sudo renice 0 -p

if [ "$FLAG_R" ]; then
    echo R course selected
    python3 main.py
elif [ "$FLAG_L" ]; then
    echo L course selected
    python3 main.py
else
    echo The course not selected
    python3 main.py
fi
