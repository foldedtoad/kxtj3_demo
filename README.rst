.. _kxtj3:

KXTJ3: Motion Sensor Monitor
#############################

Overview
********

This sample application periodically reads accelerometer data from the
KXTJ3 sensor, and displays the sensor data on the console.

Requirements
************

This sample uses the Kionix KXTJ3 system-in-package featuring a 3D digital output motion sensor.

References
**********

For more information about the LIS2DH motion sensor see
https://www.st.com/en/mems-and-sensors/lis2dh.html.

Building and Running
********************

The KXTJ3 sensors are available on a variety of boards
and shields supported by Zephyr.

Sample Output
=============

.. code-block:: console

    Polling at 0.5 Hz
    #1 @ 12 ms: x -5.387328 , y 5.578368 , z -5.463744
    #2 @ 2017 ms: x -5.310912 , y 5.654784 , z -5.501952
    #3 @ 4022 ms: x -5.349120 , y 5.692992 , z -5.463744

   <repeats endlessly every 2 seconds>
