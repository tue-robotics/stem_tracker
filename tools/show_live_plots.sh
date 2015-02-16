#/bin/bash

rqt_plot /amigo/whiskergripper/whisker_measurements/data[9]:data[10]:data[11] > /dev/null &
rqt_plot /amigo/whiskergripper/whisker_measurements/data[6]:data[7]:data[8] > /dev/null &
rqt_plot /amigo/whiskergripper/whisker_measurements/data[3]:data[4]:data[5] > /dev/null &
rqt_plot /amigo/whiskergripper/whisker_measurements/data[0]:data[1]:data[2] > /dev/null &

rqt_plot /amigo/whiskergripper/top_measurements/data[0]:data[1]:data[2]:data[3]:data[4] > /dev/null &
