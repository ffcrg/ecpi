#!/bin/bash
if [ "$1" = "" ]; then
 echo usage: $0 Verbrauch/W
 exit
fi

#echo Stromzaehler Stromsensor http://volkszaehler.org/pipermail/volkszaehler-dev/2013-February/002397.html
#echo Verbrauchszaehler Stromsensor
vzclient -u bbe93480-0d8b-11e3-b4e1-536f76fe5f15 add data value=$1 > /dev/null

#echo Verbrauch Stromzaehler mit Aufloesung 1000 Preis 0.00028
vzclient -u ab8cba30-0d8b-11e3-81a5-9531f4f32f03 add data value=$1 > /dev/null

