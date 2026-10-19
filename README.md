# mav-telemetry

A MAVLink telemetry client that keeps working when the link to the vehicle degrades or drops, plus a small fault-injection proxy for testing it against a simulated ArduPilot vehicle.

The client tracks link health from vehicle heartbeats, moves through explicit link states, re-requests its telemetry streams when the link returns, and logs both telemetry and state transitions to a JSON Lines file for later analysis.

## Why this exists

Ground software for an autonomous vehicle cannot assume the link is healthy. Links drop, packets are lost, and the vehicle may restart and forget what it was asked to send. I built this to explore how a client should behave in those conditions, and to measure it rather than assume it.

## Architecture

```
ArduPilot SITL ──(MAVLink/UDP)──▶ fault_proxy.py ──(UDP :14561)──▶ client.py ──▶ telemetry.jsonl
   (MAVProxy                         :14560                            │
    --out :14560)      ◀───────── commands (relayed back) ◀────────────┘
                                         ▲
                              stdin: blackout / drop controls
```

- **ArduPilot SITL** simulates an ArduCopter. MAVProxy forwards its telemetry to UDP port 14560 via `--out`.
- **`fault_proxy.py`** sits between the simulator and the client. It relays packets in both directions and can, on demand, drop all traffic for N seconds (blackout) or drop a fraction of packets at random.
- **`client.py`** is the telemetry client: link-health state machine, stream re-request on recovery, JSONL logging.

## Link state machine

The client uses the vehicle's HEARTBEAT message (sent at about 1 Hz) as its link-health signal.

| State | Entered when |
|---|---|
| `CONNECTING` | Client started, no heartbeat seen yet |
| `CONNECTED` | A heartbeat is received |
| `DEGRADED` | More than 2 s since the last heartbeat |
| `LOST` | More than 5 s since the last heartbeat |

Typical sequence during an outage: `CONNECTED → DEGRADED → LOST → CONNECTED`.

**Recovery.** When a heartbeat arrives and the client was not already `CONNECTED`, it re-sends `MAV_CMD_SET_MESSAGE_INTERVAL` requests for `ATTITUDE`, `GLOBAL_POSITION_INT`, and `SYS_STATUS` at 5 Hz. After an outage the vehicle may have lost or reset its stream configuration, so the client does not assume the old subscriptions still hold.

**Non-blocking loop.** The client polls with `recv_match(blocking=False)` so the state machine keeps evaluating timeouts when no data is arriving. A blocking read would hide exactly the failures this project is about.

## Design decisions

**UDP, not TCP.** Telemetry is perishable: a position from five seconds ago is worse than a missing one. TCP would retransmit lost data and hold back fresh data behind it, so after an outage the client would receive stale readings late. UDP accepts loss in exchange for low latency, and link health is tracked at the application level with heartbeats.

**Reliability belongs at the application layer.** MAVLink provides sequence numbers (to detect gaps) and command acknowledgements. The intended approach is lossy and fast for streaming data, with explicit acknowledge-and-retry only for commands that matter. This client currently sends no safety-relevant commands (only stream requests).

**No blind resend after recovery.** Anything in flight when the link died could be out of date. Resending an old command automatically is a hazard on a vehicle, so the client re-requests streams (idempotent) and nothing else.

## Setup

Requirements: Linux, Python 3, and a working [ArduPilot SITL](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html) install.

```bash
git clone https://github.com/EmmanuelAttah1/mav-telemetry.git
cd mav-telemetry
python3 -m venv .venv && . .venv/bin/activate
pip install pymavlink pytest
```

## Running it

Use three terminals.

**1. Simulator**
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --out=udp:127.0.0.1:14560
```

**2. Fault proxy**
```bash
python fault_proxy.py
```
Controls are typed into this terminal:
- `b 10`: blackout for 10 seconds
- `d 0.3`: drop 30% of packets (`d 0` to turn off)

**3. Client**
```bash
python client.py
```

Output is written to `telemetry.jsonl`. Each line is one JSON object:
```json
{"t": 1760000000.0, "kind": "link_state", "old": "CONNECTED", "new": "DEGRADED"}
{"t": 1760000000.2, "kind": "msg", "type": "ATTITUDE", "data": {"...": "..."}}
```

## Results

Measured with `OUTAGE_RUNS=10 pytest -s tests/` against ArduPilot SITL on a loopback link, using a 10 s blackout.

**Blackout recovery** (N = 10 runs): time from the end of the outage to the client returning to `CONNECTED`.

| Metric | Value |
|---|---|
| Min | 0.90 s |
| Median | 0.94 s |
| Max | 0.97 s |

**State transitions observed:** `CONNECTED -> DEGRADED -> LOST -> CONNECTED` in 10/10 runs, with telemetry messages resuming after reconnection in every run.

**Interpreting the numbers:** the vehicle sends heartbeats at about 1 Hz, so recovery is bounded by the heartbeat interval. The client reconnects on the first heartbeat after the link returns; the measured time reflects the heartbeat period, not client processing delay.

## Tests

```bash
pytest tests/
```

`tests/test_outage.py` starts the proxy and client, triggers a blackout, and asserts that the logged state transitions occur in the expected order and that the client recovers.

## Limitations and next steps

- Tested against ArduPilot SITL only, not real hardware or a real radio link.
- The proxy models blackouts and random loss, not latency, jitter, reordering, or duplication.
- Link health uses a single signal (heartbeat). A production client would also use message sequence numbers to estimate loss rate.
- Commands with acknowledgement and retry are not implemented.
- The client logs every message; a real deployment would log a reduced set at controlled rates.
- Possible next step: port the link monitor to C++.

## Project layout

```
client.py          telemetry client: state machine, recovery, logging
fault_proxy.py     UDP proxy with blackout and packet-drop injection
tests/             automated outage test
README.md
```