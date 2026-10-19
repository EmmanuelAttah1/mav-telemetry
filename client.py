import json, time
from pymavlink import mavutil

DEGRADED_AFTER = 2.0   # seconds without heartbeat
LOST_AFTER = 5.0

def run(conn="udpin:0.0.0.0:14561", log_path="telemetry.jsonl"):
    m = mavutil.mavlink_connection(conn)
    log = open(log_path, "a")
    state, last_hb = "CONNECTING", None

    def emit(kind, **kw):
        log.write(json.dumps({"t": time.time(), "kind": kind, **kw}) + "\n")
        log.flush()

    def set_state(new):
        nonlocal state
        if new != state:
            emit("link_state", old=state, new=new)
            state = new

    def resync():
        for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                       mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                       mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS):
            m.mav.command_long_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, 200_000, 0, 0, 0, 0, 0)  # 5 Hz

    while True:
        msg = m.recv_match(blocking=False)   # never block: the state machine must keep ticking
        now = time.monotonic()
        if msg and msg.get_type() != "BAD_DATA":
            t = msg.get_type()
            if t == "HEARTBEAT":
                if state != "CONNECTED":
                    resync()                 # vehicle may have lost our stream requests
                set_state("CONNECTED")
                last_hb = now
            emit("msg", type=t, data=msg.to_dict())
        else:
            time.sleep(0.005)
        if last_hb is not None:
            gap = now - last_hb
            if gap > LOST_AFTER:
                set_state("LOST")
            elif gap > DEGRADED_AFTER:
                set_state("DEGRADED")

#run()
