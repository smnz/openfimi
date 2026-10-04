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


def test_retarget_mid_move_and_route_after_fly_to():
    # The real aircraft refuses a new fly-to mid-move (21) and a route upload
    # during a fly-to (41); ManualFlight uses restartable routes instead.
    rig = flying_rig()
    try:
        d = rig.drone
        events = []
        m = ManualFlight(d, on_event=events.append).start()
        hold(m, 1.2, pitch=1.0)
        hold(m, 1.2, roll=1.0)  # change of direction mid-move
        assert not any("refused" in e for e in events), events
        assert rig.sim.log.count("3/32") >= 2
        m.stop()
    finally:
        rig.close()


def test_release_stops_a_climb_and_first_move_after_fly_to_is_accepted():
    rig = flying_rig()
    try:
        d = rig.drone
        # a fly-to that has just arrived (the sim, like the aircraft, then refuses routes)
        s = d.state.sport
        d.fly_to(s.lat, s.lon, s.height_m + 1, 1.0)
        d.wait_until(lambda st: st.sport.ground_speed_ms < 0.1 and rig.sim.activity == "hover", 5)
        events = []
        # 30 s lookahead: a +60 m target, still climbing when released
        m = ManualFlight(d, lookahead_s=30.0, max_alt_m=200, on_event=events.append).start()
        hold(m, 0.6, throttle=1.0)
        n_log = len(rig.sim.log)
        m.centre()
        time.sleep(1.0)
        assert not any("refused" in e for e in events), events  # fly-to exited first
        # releasing a climb stops the route (the real aircraft kept climbing)
        assert "manual: holding position" in events
        assert "3/35" in rig.sim.log[n_log:]
        m.stop()
    finally:
        rig.close()


def test_bridge_emergency_stops_manual_flight():
    rig = flying_rig()
    try:
        d = rig.drone
        events = []
        m = ManualFlight(d, on_event=events.append).start()
        d._on_notice({"event": "emergency_rth", "source": "bridge", "stage": "activated"})
        n = len(rig.sim.log)
        hold(m, 1.0, pitch=1.0)  # inputs after the emergency are ignored
        assert any("emergency" in e for e in events), events
        assert "3/32" not in rig.sim.log[n:]
        m.stop(halt=False)
    finally:
        rig.close()
