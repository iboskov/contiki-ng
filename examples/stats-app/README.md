USAGE:
    
    - project-conf.h 
        - you must first define RF2XX_CONF_STATS 1 for colecting statistics in radio driver
        - then define buffer capacity - if values are printed every 10s, buffer of 20 packets should do
        - define density of measurements of background noise...possible measurement every 1, 2 or 10 ms
        - turn off logs of all other modules, so they do not disturb our serial monitor

    - serial_monitor.py
        $ python3 serial_monitor.py -p ttyUSB0 -o node_stats.txt
        $ python3 serial_monitor.py -p ttyUSB1 -o root_stats.txt -r

WORKING: 
    
    First turn on (flash) all Vesna devices - they will wait for start command.
    Then run "Serial_monitor". It will automatically send start command - the nodes will start to logg statistics
    and send them back to serial monitor.
    
    App closes automatically after 10 minutes or after max 7000 lines are stored. User can
    stop the app at any time with "Ctrl+C".
