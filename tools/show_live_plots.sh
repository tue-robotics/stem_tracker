#/bin/bash

rqt_plot /amigo/whiskergripper/measurements/data[0]:data[1]:data[2]:data[3] > /dev/null &
rqt_plot /amigo/whiskergripper/measurements/data[4]:data[5]:data[6]:data[7]:data[8] > /dev/null &

#rqt_plot /amigo/whiskergripper/measurements/data[0] > /dev/null &
