from pymavlink import mavutil

m = mavutil.mavlink_connection("udpin:0.0.0.0:14561")
m.wait_heartbeat()
print("heartbeat from system", m.target_system)

while True:
    msg = m.recv_match(blocking=True, timeout=1)
    if msg:
        print(msg.get_type())
