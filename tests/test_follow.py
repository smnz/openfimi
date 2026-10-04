from openfimi.follow import GimbalFollower
from openfimi.mission import Mission, PointAction, Waypoint

LAT, LON = -43.5321, 172.6362
DEG_PER_M = 1 / 111320


def route(pitches, action_at=()):
    wps = []
    for i, p in enumerate(pitches):
        act = PointAction.photo() if i in action_at else PointAction()
        wps.append(Waypoint(LAT + (i + 1) * 60 * DEG_PER_M, LON, 10, act, gimbal_pitch_deg=p))
    return Mission(wps, speed_ms=10, finish=4)


class _Stub:
    state = None
    link = None


def test_rule_holds_until_departure_then_steps():
    m = route([-30, -60, -90])
    f = GimbalFollower(_Stub(), m)
    wp = [(w.lat, w.lon) for w in m.waypoints]
    assert f.target_pitch(0, (LAT, LON)) == -30  # climbing to wp0
    assert f.target_pitch(1, wp[0]) == -30  # just reached wp0: keep its pitch for its action
    mid = ((wp[0][0] + wp[1][0]) / 2, LON)
    assert f.target_pitch(1, mid) == -60  # left wp0: next waypoint's pitch
    assert f.target_pitch(3, wp[2]) == -90  # finished


def test_rule_interpolates():
    m = route([-30, -60, -90])
    f = GimbalFollower(_Stub(), m, mode="interpolate")
    wp = [(w.lat, w.lon) for w in m.waypoints]
    mid = ((wp[0][0] + wp[1][0]) / 2, LON)
    assert abs(f.target_pitch(1, mid) - (-45)) < 1.0


def test_pitch_clamped():
    f = GimbalFollower(_Stub(), route([-120, 30]))
    assert f.pitches == [-90.0, 10.0]


def test_fly_route_drives_gimbal_in_sim():
    from test_link import Rig

    rig = Rig()
    try:
        rig.sim.home = (LAT, LON)
        rig.sim.lat, rig.sim.lon = LAT, LON
        d = rig.drone
        d.wait_for_telemetry(2)
        d.wait_until(lambda s: s.battery and s.signal and s.errors, 3)
        m = route([-30, -60, -90], action_at=(1,))
        events = []
        res = d.fly_route(m, on_event=events.append, timeout=60)
        assert res.get("landed"), events
        # the photo at waypoint 1 was taken at waypoint 1's pitch
        assert rig.sim.photos == [(1, -60.0)], (rig.sim.photos, events)
        # the gimbal visited each waypoint's pitch in order
        seq = [
            p for i, p in enumerate(rig.sim.gimbal_log) if i == 0 or p != rig.sim.gimbal_log[i - 1]
        ]
        assert seq[:3] == [-30.0, -60.0, -90.0], seq
    finally:
        rig.close()
