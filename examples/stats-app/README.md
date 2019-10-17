USAGE:
    
    - project-conf.h 
        - you must first define RF2XX_CONF_STATS 1 for colecting statistics in radio driver
        - then define buffer capacity - if values are printed every 10s, buffer of 20 packets should do
        - turn off logs of all other modules, so they do not disturb our serial monitor

    - stats-app.c
        - define capacity of buffer for measuring background noise (STATS_BG_NOISE_BUFF_CAPACITYI)
          If values are printed every 1 second, buffer of 1000 measurments will do

    - serial_monitor.py
        $ python3 serial_monitor.py -p ttyUSB0 -o node_stats.txt
        $ python3 serial_monitor.py -p ttyUSB1 -o root_stats.txt -r

WORKING: 
    First, run the "serial_monitor", then reset (or flash) Vesna device. Monitor will automaticly search
    for start sequence (character '*') and will monitor its input until stop sequence (character '=') 
    or until 'LINES_TO_READ' lines are stored.