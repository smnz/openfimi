from openfimi.follow import GimbalFollower
from openfimi.mission import GimbalMode, Mission, PointAction, Waypoint

LAT, LON = -43.5321, 172.6362
DEG_PER_M = 1 / 111320
NONE, BEFORE, ARRIVAL = GimbalMode.NONE, GimbalMode.BEFORE_ARRIVAL, GimbalMode.ON_ARRIVAL


def route(spec, spacing_m=60, action_at=()):
    """spec: [(pitch, mode), ...]; waypoints spacing_m apart going north."""
    wps = []
    for i, (p, mode) in enumerate(spec):
        act = PointAction.photo() if i in action_at else PointAction()
        wps.append(
            Waypoint(
                LAT + (i + 1) * spacing_m * DEG_PER_M,
                LON,
                10,
                act,
                gimbal_pitch_deg=p,
                gimbal_mode=mode,
            )
        )
    return Mission(wps, speed_ms=10, finish=4)


class _Stub:
    state = None
    link = None


def at(m_north):
    return (LAT + m_north * DEG_PER_M, LON)


def test_none_never_commands():
    f = GimbalFollower(_Stub(), route([(-90, NONE), (-90, NONE)]))
    assert not f.active
    for reached, pos in ((0, at(0)), (1, at(60)), (1, at(90)), (2, at(120))):
        assert f.decide(reached, pos) is None


def test_on_arrival_fires_when_reached_only():
    f = GimbalFollower(_Stub(), route([(0, NONE), (-60, ARRIVAL), (0, NONE)]))
    assert f.decide(0, at(30)) is None
    assert f.decide(1, at(60)) is None  # wp0 reached (mode none)
    assert f.decide(1, at(100)) is None  # flying to wp1
    assert f.decide(2, at(120)) == 1  # wp1 reached
    assert f.decide(2, at(150)) is None  # once only


def test_before_arrival_short_leg_fires_on_departure():
    # 60 m at 10 m/s = 6 s < 15 s
    f = GimbalFollower(_Stub(), route([(-30, ARRIVAL), (-60, BEFORE)]))
    f.decide(0, at(0))
    assert f.decide(1, at(60)) == 0  # wp0 on arrival
    assert f.decide(1, at(61)) is None  # still at wp0 (its photo)
    assert f.decide(1, at(65)) == 1  # left wp0: set wp1's pitch now


def test_before_arrival_long_leg_fires_15s_out():
    # 300 m at 10 m/s = 30 s
    f = GimbalFollower(_Stub(), route([(0, NONE), (-60, BEFORE)], spacing_m=300))
    f.decide(1, at(300))  # wp0 reached
    assert f.decide(1, at(310)) is None  # 29 s out
    assert f.decide(1, at(440)) is None  # 16 s out
    assert f.decide(1, at(460)) == 1  # 14 s out
    assert f.decide(1, at(500)) is None  # once only


def test_missed_before_arrival_fires_on_arrival():
    f = GimbalFollower(_Stub(), route([(0, NONE), (-45, BEFORE)], spacing_m=300))
    assert f.decide(2, at(600)) == 1  # joined late: catch up at arrival


def test_pitch_clamped():
    f = GimbalFollower(_Stub(), route([(-120, ARRIVAL), (30, ARRIVAL)]))
    assert f.pitches == [-90.0, 10.0]


def test_plan_text():
    f = GimbalFollower(_Stub(), route([(0, NONE), (-60, ARRIVAL), (-90, BEFORE)]))
    plan = f.plan()
    assert plan[0].startswith("wp1: -60 deg on arrival")
    assert "on leaving wp1" in plan[1]


def test_fly_route_drives_gimbal_in_sim():
    from test_link import Rig

    rig = Rig()
    try:
        rig.sim.home = (LAT, LON)
        rig.sim.lat, rig.sim.lon = LAT, LON
        d = rig.drone
        d.wait_for_telemetry(2)
        d.wait_until(lambda s: s.battery and s.signal and s.errors, 3)
        m = route([(-30, BEFORE), (-60, BEFORE), (0, NONE), (-90, ARRIVAL)], action_at=(1,))
        events = []
        res = d.fly_route(m, on_event=events.append, timeout=60)
        assert res.get("landed"), events
        assert rig.sim.photos == [(1, -60.0)], (rig.sim.photos, events)  # photo at its own pitch
        seq = [
            p for i, p in enumerate(rig.sim.gimbal_log) if i == 0 or p != rig.sim.gimbal_log[i - 1]
        ]
        assert seq == [-30.0, -60.0, -90.0], (seq, events)  # wp2 (none) left it alone
    finally:
        rig.close()
