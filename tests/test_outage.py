"""
Outage recovery test.

Starts fault_proxy.py and the telemetry client as subprocesses, waits for the
client to reach CONNECTED, triggers a blackout through the proxy's stdin, then
checks the client's JSONL log for:

  1. the link states DEGRADED -> LOST -> CONNECTED, in that order
  2. recovery time (end of blackout to the CONNECTED transition) under a limit
  3. telemetry messages flowing again after reconnection

Prerequisite: ArduPilot SITL must already be running, started with
    sim_vehicle.py -v ArduCopter --console --out=udp:127.0.0.1:14560
and nothing else (e.g. a manually started proxy or client) may be using
UDP ports 14560 or 14561.

Run once:            pytest -s tests/
Collect statistics:  OUTAGE_RUNS=10 pytest -s tests/
"""

import json
import os
import statistics
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent

OUTAGE_S = 10.0            # blackout length; must exceed the client's LOST threshold (5 s)
CONNECT_TIMEOUT_S = 30.0   # time allowed for the first CONNECTED
SEQUENCE_TIMEOUT_S = 30.0  # time allowed (after blackout starts) for the full state sequence
MAX_RECOVERY_S = 3.0       # heartbeats arrive ~1 Hz, so recovery should be about 1 s
RUNS = int(os.environ.get("OUTAGE_RUNS", "1"))

RESULTS = []


def read_log(path):
    """Parse the JSONL log, ignoring a partially written last line."""
    events = []
    if not path.exists():
        return events
    for line in path.read_text().splitlines():
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return events


def link_states(events, after=0.0):
    """Return [(timestamp, new_state), ...] for link transitions after a time."""
    return [(e["t"], e["new"]) for e in events
            if e.get("kind") == "link_state" and e["t"] > after]


def wait_for(predicate, timeout, interval=0.1):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(interval)
    return None


@pytest.fixture(scope="module", autouse=True)
def summarise():
    yield
    if RESULTS:
        print("\n--- recovery time summary ---")
        print(f"runs:   {len(RESULTS)}")
        print(f"min:    {min(RESULTS):.2f} s")
        print(f"median: {statistics.median(RESULTS):.2f} s")
        print(f"max:    {max(RESULTS):.2f} s")


@pytest.fixture
def stack(tmp_path):
    """Start the proxy and client; yield (proxy_process, log_path); clean up."""
    log = tmp_path / "telemetry.jsonl"

    proxy = subprocess.Popen(
        [sys.executable, "fault_proxy.py"],
        cwd=ROOT, stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, text=True,
    )
    client = subprocess.Popen(
        [sys.executable, "-c",
         f"import client; client.run(log_path={str(log)!r})"],
        cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    try:
        yield proxy, log
    finally:
        for p in (client, proxy):
            p.terminate()
        for p in (client, proxy):
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()


@pytest.mark.parametrize("run", range(RUNS))
def test_outage_recovery(stack, run):
    proxy, log = stack

    # 1. Wait until the client is connected to the (simulated) vehicle.
    connected = wait_for(
        lambda: any(s == "CONNECTED" for _, s in link_states(read_log(log))),
        CONNECT_TIMEOUT_S,
    )
    assert connected, (
        "Client never reached CONNECTED. Is SITL running with "
        "--out=udp:127.0.0.1:14560, and are ports 14560/14561 free?"
    )

    # 2. Trigger the blackout.
    t_start = time.time()
    proxy.stdin.write(f"b {OUTAGE_S}\n")
    proxy.stdin.flush()
    outage_end = t_start + OUTAGE_S

    # 3. Wait for DEGRADED -> LOST -> CONNECTED after the blackout began.
    def sequence_complete():
        states = [s for _, s in link_states(read_log(log), after=t_start)]
        return states if len(states) >= 3 else None

    states = wait_for(sequence_complete, SEQUENCE_TIMEOUT_S)
    assert states, (
        "Full state sequence not seen. Transitions after blackout: "
        f"{[s for _, s in link_states(read_log(log), after=t_start)]}"
    )
    assert states[:3] == ["DEGRADED", "LOST", "CONNECTED"], \
        f"Unexpected state order: {states}"

    # 4. Measure recovery: end of blackout -> CONNECTED transition.
    transitions = link_states(read_log(log), after=t_start)
    t_reconnected = transitions[2][0]
    recovery = t_reconnected - outage_end
    RESULTS.append(recovery)
    print(f"\nrun {run}: recovery {recovery:.2f} s")
    assert recovery < MAX_RECOVERY_S, \
        f"Recovery took {recovery:.2f} s (limit {MAX_RECOVERY_S} s)"

    # 5. Telemetry must be flowing again after reconnection.
    def telemetry_resumed():
        return any(
            e.get("kind") == "msg" and e["t"] > t_reconnected
            and e.get("type") in ("ATTITUDE", "GLOBAL_POSITION_INT", "SYS_STATUS")
            for e in read_log(log)
        )

    assert wait_for(telemetry_resumed, 10.0), \
        "No telemetry messages logged after reconnection"
