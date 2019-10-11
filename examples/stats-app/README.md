A simple app that shows the statistics of rf2xx radio!

USAGE:
    * project-conf.h 
        - you must first define RF2XX_CONF_STATS 1 for colecting statistics in radio driver
        - then define buffer capacity - if values are printed every 10s, buffer of 20 packets should do
        - turn off logs of all other modules, so they do not disturb our serial monitor

    * stats-app.c
        - define capacity of buffer for measuring background noise (STATS_BG_NOISE_BUFF_CAPACITYI)
          If values are printed every 1 second, buffer of 1000 measurments will do

    * serial_monitor.py
        - define maximum numbers of lines to read by serial monitor (LINES_TO_READ)
        -$ python3 serial_monitor.py -o rf2xx_stats.txt -p ttyUSB1
         This will start serial monitor on port ttyUSB1 and store input into file rf2xx_stats.txt

WORKING: 
    First, run the serial monitor, then reset (or flash) Vesna device. Monitor will automaticly search
    for start sequence (character '*') and will monitor its input until stop sequence (character '=') 
    or until LINES_TO_READ lines are stored.