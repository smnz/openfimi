import math
import time

from test_link import Rig

from openfimi.follow import ground_distance
from openfimi.manual import ManualFlight


def flying_rig():
    rig = Rig()
    d = rig.drone
    d.wait_for_telemetry(2)
    d.takeoff()
    assert d.wait_until(lambda s: s.heart.flight_phase == 3 and s.sport.height_m > 2, 10)
    return rig


def hold(m, seconds, **inputs):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        m.set(**inputs)
        time.sleep(0.1)


def test_forward_moves_along_locked_heading_and_release_stops():
    rig = flying_rig()
    try:
        d = rig.drone
        start = (d.state.sport.lat, d.state.sport.lon)
        events = []
        m = ManualFlight(d, on_event=events.append).start()
        heading = m.frame_yaw
        hold(m, 1.5, pitch=1.0)
        moved = ground_distance(start, (d.state.sport.lat, d.state.sport.lon))
        assert moved > 3, (moved, events)
        # direction of travel matches the locked frame
        s = d.state.sport
        dn = (s.lat - start[0]) * 111320
        de = (s.lon - start[1]) * 111320 * math.cos(math.radians(s.lat))
        bearing = math.degrees(math.atan2(de, dn))
        assert abs(((bearing - heading) + 180) % 360 - 180) < 15
        m.centre()
        time.sleep(1.0)
        assert not m.moving and "manual: holding position" in events
        p = (d.state.sport.lat, d.state.sport.lon)
        time.sleep(0.6)
        assert ground_distance(p, (d.state.sport.lat, d.state.sport.lon)) < 0.5  # hovering
        m.stop()
    finally:
        rig.close()


def test_up_climbs_and_deadman_halts():
    rig = flying_rig()
    try:
        d = rig.drone
        h0 = d.state.sport.height_m
        m = ManualFlight(d).start()
        hold(m, 1.5, throttle=1.0)
        assert d.state.sport.height_m > h0 + 2
        # no updates at all: the dead-man stops the climb
        time.sleep(1.5)
        assert not m.moving
        m.stop()
    finally:
        rig.close()


def test_yaw_is_reported_unavailable():
    rig = flying_rig()
    try:
        events = []
        m = ManualFlight(rig.drone, on_event=events.append).start()
        m.set(yaw=1.0)
        assert any("yaw is not available" in e for e in events)
        m.stop()
    finally:
        rig.close()
