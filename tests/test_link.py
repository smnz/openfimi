import struct
import threading
import time

import pytest

from openfimi import Drone, commands, telemetry
from openfimi.framing import encode_inner, encode_outer
from openfimi.link import AckTimeout, Link
from openfimi.mission import Mission, PointAction, Waypoint
from openfimi.modules import Module
from openfimi.sim import SimAircraft
from openfimi.transport import LoopbackTransport


class Rig:
    """A Drone wired to a SimAircraft through a loopback transport."""

    def __init__(self):
        self.t = LoopbackTransport()
        self.sim = SimAircraft(rate_hz=50)
        self.sim.out = self.t.feed
        self.t.on_write = self.sim.receive
        self.stop = threading.Event()
        threading.Thread(target=self._physics, daemon=True).start()
        self.drone = Drone(self.t, init_camera=False).connect()

    def _physics(self):
        while not self.stop.is_set():
            self.sim.step(0.2)  # 10x real time
            time.sleep(0.02)

    def close(self):
        self.stop.set()
        self.drone.close()


@pytest.fixture
def rig():
    r = Rig()
    yield r
    r.close()


def test_telemetry_flows_and_decodes(rig):
    d = rig.drone
    assert d.wait_for_telemetry(2)
    assert d.wait_until(lambda s: s.battery and s.home and s.gimbal, 2)
    lat, lon, h = d.state.position
    assert abs(lat - rig.sim.home[0]) < 1e-6 and abs(lon - rig.sim.home[1]) < 1e-6
    assert d.state.battery.percent >= 99
    assert d.state.flying is False


def test_takeoff_land_cycle(rig):
    d = rig.drone
    d.wait_for_telemetry(2)
    r = d.takeoff()
    assert r.ok and r.frame.seq == r.frame.seq
    assert d.wait_until(lambda s: s.sport and s.sport.height_m > 1.0, 5)
    assert d.state.flying
    assert d.takeoff().code == 1  # already flying: rejected
    assert d.land().ok
    assert d.wait_until(lambda s: s.heart and not s.heart.flying, 5)


def test_mission_upload_readback_and_fly(rig):
    d = rig.drone
    d.wait_for_telemetry(2)
    lat0, lon0 = rig.sim.home
    m = Mission(
        [
            Waypoint(lat0 + 0.0002, lon0, 10, PointAction.photo()),
            Waypoint(lat0 + 0.0002, lon0 + 0.0003, 12),
        ],
        speed_ms=10,
    )
    d.upload_mission(m)
    pts = d.read_mission()
    assert [round(p.alt_m) for p in pts] == [10, 12]
    assert abs(pts[1].lon - (lon0 + 0.0003)) < 1e-9
    d.takeoff()
    d.wait_until(lambda s: s.sport and s.sport.height_m > 1.0, 5)
    assert d.start_mission().ok
    assert d.wait_until(lambda s: s.navigation and s.navigation.waypoint == 1, 10)


def test_gimbal_and_reply_message(rig):
    d = rig.drone
    d.wait_for_telemetry(2)
    d.gimbal_pitch(-60)
    assert d.wait_until(lambda s: s.gimbal and s.gimbal.pitch_raw == -6000, 2)


def test_sticks_deadman(rig):
    d = rig.drone
    d.wait_for_telemetry(2)
    d.takeoff()
    d.wait_until(lambda s: s.sport and s.sport.height_m > 1.0, 5)
    s = d.sticks.start()
    s.deadman = 0.3
    s.set(throttle=1.0)
    time.sleep(0.15)
    assert rig.sim.sticks[2] == 0  # throttle up reads low on the RC
    time.sleep(0.6)  # no updates: the streamer must re-centre
    assert rig.sim.sticks[2] == 512
    s.stop()


def test_retransmit_then_timeout():
    t = LoopbackTransport()
    link = Link(t).start()
    try:
        cmd = commands.takeoff()
        fast = type(cmd)(cmd.dest, cmd.payload, timeout=0.05, retries=2, name="t")
        with pytest.raises(AckTimeout):
            link.request(fast, timeout=2)
        assert len(t.sent) == 3  # first send + 2 retries, same bytes
        assert len(set(t.sent)) == 1
        assert link.stats.retransmits == 2
    finally:
        link.stop()


def test_ack_matches_on_group_msg_seq():
    t = LoopbackTransport()
    link = Link(t).start()
    try:
        fut = link.send(commands.land())
        seq = t.sent[0][5 + 8] | t.sent[0][5 + 9] << 8
        # wrong seq is ignored, right one completes
        t.feed(encode_outer(encode_inner(Module.FC, Module.GCS, seq + 1, bytes([3, 21, 0, 0]))))
        t.feed(encode_outer(encode_inner(Module.FC, Module.GCS, seq, bytes([3, 21, 0x20, 0]))))
        r = fut.result(1)
        assert r.code == 2 and not r.ok
    finally:
        link.stop()


def test_decoders_reject_short_bodies():
    with pytest.raises(ValueError):
        telemetry.FcSportState.decode(bytes(10))
    m = telemetry.FcSportState.decode(
        struct.pack("<ddfhhhhhbbfh", 2.0, 1.0, 3.0, 0, 0, 15, -20, 900, 0, 0, 50.0, 0)
    )
    assert (m.lat, m.lon, m.roll_deg, m.pitch_deg, m.yaw_deg) == (1.0, 2.0, 1.5, -2.0, 90.0)


def test_fly_to_sets_target_then_starts(rig):
    d = rig.drone
    d.wait_for_telemetry(2)
    assert d.send(commands.fly_to_start()).code == 30  # no target yet, as in flight
    d.takeoff()
    d.wait_until(lambda s: s.sport and s.sport.height_m > 1.0, 5)
    lat0, lon0 = rig.sim.home
    assert d.fly_to(lat0 + 0.0002, lon0, 30, speed_ms=10).ok
    assert d.wait_until(lambda s: s.sport.height_m > 25 and s.sport.home_distance_m > 15, 15)


def test_wait_until_ready_waits_for_gps_and_home(rig):
    from openfimi.drone import PreflightError

    rig.sim.satellites, rig.sim.home_set = 4, False
    d = rig.drone
    d.wait_for_telemetry(2)
    d.wait_until(
        lambda s: s.battery and s.errors and s.signal.satellites == 4 and s.home.lat == 0, 5
    )
    assert "4/10 satellites" in d.preflight()
    with pytest.raises(PreflightError, match="not ready after"):
        d.wait_until_ready(1.5, settle_s=0.5)

    def improve():
        time.sleep(0.8)
        rig.sim.satellites = 12
        time.sleep(0.5)
        rig.sim.home_set = True

    threading.Thread(target=improve, daemon=True).start()
    events = []
    d.wait_until_ready(10, settle_s=0.5, on_event=events.append)
    assert events[-1].startswith("ready: 12 satellites"), events
    # a stricter fix keeps waiting
    with pytest.raises(PreflightError, match="12/16 satellites"):
        d.wait_until_ready(1.5, min_satellites=16, settle_s=0.5)


def test_wait_until_ready_fails_fast_on_overheat(rig):
    from openfimi.drone import PreflightError

    d = rig.drone
    d.wait_for_telemetry(2)
    d.wait_until(lambda s: s.errors, 3)
    d.state.errors.a |= 1 << 17  # as the real aircraft reported when too hot
    t0 = time.monotonic()
    with pytest.raises(PreflightError, match="temperature"):
        d.wait_until_ready(30)
    assert time.monotonic() - t0 < 1.0


def test_wait_until_ready_requires_settled_on_ground(rig):
    from openfimi.drone import PreflightError

    d = rig.drone
    d.wait_for_telemetry(2)
    d.wait_until(lambda s: s.battery and s.signal and s.errors and s.home, 3)
    rig.sim.carried = True  # switched on while walking to the launch spot
    events = []
    with pytest.raises(PreflightError, match="not level|moving|being moved"):
        d.wait_until_ready(1.5, settle_s=1.0, on_event=events.append)

    def put_down():
        time.sleep(0.5)
        rig.sim.carried = False

    threading.Thread(target=put_down, daemon=True).start()
    t0 = time.monotonic()
    d.wait_until_ready(10, settle_s=1.0, on_event=events.append)
    assert time.monotonic() - t0 >= 1.4  # 0.5 s carried + 1 s settling
    assert events[-1].endswith("settled"), events
