from pymavlink import mavutil
import time

connection = mavutil.mavlink_connection('/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_48003D001851303139323937-if00')


connection.wait_heartbeat()


print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_system))

while True:
    msg = connection.recv_match()
    if not msg:
        continue
    if msg.get_type() == 'HEARTBEAT':
        print("msg: %s %msg")
    