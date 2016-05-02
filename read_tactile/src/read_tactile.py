#!/usr/bin/env python
# Example of interaction with a BLE UART device using a UART service
# implementation.
# Author: Tony DiCola

PKG = 'read_tactile'
import roslib; roslib.load_manifest(PKG)
import thread
from std_msgs.msg import String
import rospy

import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART

ble = Adafruit_BluefruitLE.get_provider()
ble.initialize()
sensor_value = ""

def connect_device():
    global sensor_value

     # Clear any cached data because both bluez and CoreBluetooth have issues with
    # caching data and it going stale.
    ble.clear_cached_data()

    # Get the first available BLE network adapter and make sure it's powered on.
    adapter = ble.get_default_adapter()
    adapter.power_on()
    print('Using adapter: {0}'.format(adapter.name))

    # Disconnect any currently connected UART devices.  Good for cleaning up and
    # starting from a fresh state.
    print('Disconnecting any connected UART devices...')
    UART.disconnect_devices()

    # Scan for UART devices.
    print('Searching for UART device...')
    try:
        adapter.start_scan()
        # Search for the first UART device found (will time out after 60 seconds
        # but you can specify an optional timeout_sec parameter to change it).
        device = UART.find_device()
        if device is None:
            raise RuntimeError('Failed to find UART device!')
    finally:
        # Make sure scanning is stopped before exiting.
        adapter.stop_scan()

    print('Connecting to device...')
    device.connect()  # Will time out after 60 seconds, specify timeout_sec parameter
                      # to change the timeout.

    # Once connected do everything else in a try/finally to make sure the device
    # is disconnected when done.
    try:
        # Wait for service discovery to complete for the UART service.  Will
        # time out after 60 seconds (specify timeout_sec parameter to override).
        print('Discovering services...')
        UART.discover(device)

        # Once service discovery is complete create an instance of the service
        # and start interacting with it.
        uart = UART(device)

        # Write a string to the TX characteristic.
        uart.write('Hello world!\r\n')
        print("Sent 'Hello world!' to the device.")

         # Now wait up to one minute to receive data from the device.
        print('Waiting up to 60 seconds to receive data from the device...')
        received = uart.read(timeout_sec=60)
        data_received = ""
        while(received != "Disconnect") and (not rospy.is_shutdown()):
            received = uart.read(timeout_sec=60)
            if received is not None:
                # Received data, print it out.
                if data_received == "":
                    if("+" in received):
                        start_index = received.index('+')
                        data_received = received[start_index:]

                elif data_received.startswith("+") and data_received.endswith("/"):
                    #print "Data Received " + data_received
                    sensor_value = data_received
                    #sensor_pub.publish(data_received)
                    data_received = ""

                elif data_received.startswith("+") and ("/" in data_received):
                    #print('Received: {0}'.format(data_received))
                    end_index = data_received.index('/') + 1
                    data_received = data_received[:end_index]
                    #print "Data Received " + data_received
                    sensor_value = data_received
                    #sensor_pub.publish(data_received)
                    data_received = ""

                elif data_received.startswith("+") and ("/" not in data_received):
                    data_received = data_received + received

                else:
                    pass

                sensor_pub.publish(sensor_value)
                
                #print('Received: {0}'.format(received))
                print "Data received: " + data_received
            else:
                # Timeout waiting for data, None is returned.
                received = "Disconnect"
                print('Received no data!')
            r.sleep()
    finally:
        # Make sure device is disconnected on exit.
        device.disconnect()

if __name__ == '__main__':
    sensor_pub = rospy.Publisher('tactile_sensor', String, queue_size = 10)
    rospy.init_node('read_tactile_sensor', anonymous=True)
    r = rospy.Rate(10)
    #print_sensor()
    #thread.start_new_thread( print_sensor, ())
    ble.run_mainloop_with(connect_device)

