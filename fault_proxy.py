import socket, random, threading, time

LISTEN = ("127.0.0.1", 14560)
CLIENT = ("127.0.0.1", 14561)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(LISTEN)
upstream = None
blackout_until = 0.0
drop_prob = 0.0

def control():
    global blackout_until, drop_prob
    for line in iter(input, None):
        print(line.partition(" "))
        cmd, _, val = line.partition(" ")
        if cmd == "b":   # b 10  -> blackout for 10 seconds
            blackout_until = time.monotonic() + float(val)
        elif cmd == "d": # d 0.3 -> drop 30% of packets
            drop_prob = float(val)

threading.Thread(target=control, daemon=True).start()

while True:
    data, addr = s.recvfrom(4096)
    if time.monotonic() < blackout_until or random.random() < drop_prob:
        continue
    if addr == CLIENT:
        if upstream:
            s.sendto(data, upstream)
    else:
        upstream = addr
        s.sendto(data, CLIENT)
