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


def test_yaw_turns_via_poi_route_then_forward_follows_new_heading():
    rig = flying_rig()
    try:
        d = rig.drone
        events = []
        m = ManualFlight(d, on_event=events.append).start()
        h0 = m.heading
        hold(m, 1.0, yaw=1.0)  # ~30 deg/s clockwise
        turned = ((m.heading - h0) + 180) % 360 - 180
        assert 20 < turned < 45, turned
        assert d.wait_until(
            lambda s: abs(((s.sport.yaw_deg - m.heading) + 180) % 360 - 180) < 5, 3
        ), (d.state.sport.yaw_deg, m.heading, events)
        assert "3/32" in rig.sim.log and "3/36" in rig.sim.log  # one-point route with a POI
        m.centre()
        time.sleep(0.5)
        start = (d.state.sport.lat, d.state.sport.lon)
        hold(m, 1.5, pitch=1.0)
        s = d.state.sport
        dn = (s.lat - start[0]) * 111320
        de = (s.lon - start[1]) * 111320 * math.cos(math.radians(s.lat))
        bearing = math.degrees(math.atan2(de, dn))
        assert abs(((bearing - m.heading) + 180) % 360 - 180) < 15  # forward = new heading
        m.stop()
    finally:
        rig.close()


def test_turn_target_creeps_when_configured():
    rig = flying_rig()
    try:
        d = rig.drone
        m = ManualFlight(d, yaw_creep_m=4.0)
        m.heading = 90.0  # east
        lat, lon, alt, speed, translating = m._target((0.0, 0.0, 0.0))
        assert not translating
        m.start()
        m.heading = 90.0
        start = (d.state.sport.lat, d.state.sport.lon)
        hold(m, 1.0, yaw=0.01)  # below the input threshold: no turn command
        hold(m, 0.6, yaw=1.0)
        time.sleep(1.5)
        moved = ground_distance(start, (d.state.sport.lat, d.state.sport.lon))
        assert 1.0 < moved < 8.0, moved  # crept a few metres along the new heading
        m.stop()
    finally:
        rig.close()
