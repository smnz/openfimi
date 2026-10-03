import struct

import pytest

from openfimi import commands
from openfimi.mission import (
    FinishAction,
    LostAction,
    Mission,
    PointAction,
    Rotation,
    Waypoint,
    mission_from_dict,
)
from openfimi.modules import Module

# Produced by a Java transcription of the app's FcCollection.e0 (scratch harness).
JAVA_WAYPOINT = bytes.fromhex(
    "0324000001030000e3361ac05b946540ce1951da1bc445c031018cf16cee3200000003200000040114ae47e17a"
    "946540c3f5285c8fc245c07b00"
)


def test_waypoint_frame_matches_app_encoder():
    wps = [
        Waypoint(0, 0, 10),
        Waypoint(
            -43.5321,
            172.6362,
            30.5,
            yaw_deg=-37.6,
            gimbal_pitch_deg=-45,
            poi=(-43.52, 172.64, 12.3),
            rotation=Rotation.CCW,
        ),
        Waypoint(0, 0, 10),
    ]
    m = Mission(wps, speed_ms=5, finish=FinishAction.RETURN_HOME, rc_lost=LostAction.CONTINUE)
    cmds = m.upload_commands()
    assert len(cmds) == 6
    assert cmds[1].payload == JAVA_WAYPOINT
    assert cmds[1].dest == Module.FC


def test_action_frames_match_app_presets():
    a = commands  # noqa: F841
    from openfimi.mission import action_frame

    p = action_frame(PointAction.hover_then_photo(), 2, 5).payload
    assert len(p) == 56 and p[:6] == bytes([3, 37, 0, 0, 2, 5])
    assert (p[8], p[9], p[24], p[25], p[26]) == (1, 2, 5, 1, 1)
    p = action_frame(PointAction.photo(3), 0, 1).payload
    assert (p[8], p[9], p[24], p[25], p[26]) == (2, 0, 1, 3, 0)


def test_mission_validation():
    with pytest.raises(ValueError):
        Mission([]).validate()
    with pytest.raises(ValueError):
        Mission([Waypoint(0, 0, 10)], speed_ms=20).validate()


def test_mission_from_dict():
    m = mission_from_dict(
        {
            "speed_ms": 4,
            "heading": "route",
            "finish": "return_home",
            "waypoints": [
                {"lat": 1, "lon": 2, "alt_m": 20, "action": "photo", "poi": [1.1, 2.1, 5]}
            ],
        }
    )
    p = m.upload_commands()[0].payload
    assert p[30] == 40 and p[35] & 0x0F == 2 and p[38] == 4 and p[34] & 1 == 1


def test_simple_payloads():
    assert commands.takeoff().payload == bytes([3, 16, 0, 0])
    assert commands.land().payload == bytes([3, 21, 0, 0])
    assert commands.return_home().payload == bytes([3, 26])
    assert commands.mission_start().payload == bytes([3, 32])
    assert commands.take_photo().dest == Module.CAMERA
    assert commands.set_fpv_stream().payload == bytes.fromhex("027200000100000000050000d0020000")


def test_gimbal_pitch_layout():
    c = commands.gimbal_pitch(-45.5, rate=20000)
    p = c.payload
    assert c.dest == Module.GIMBAL and len(p) == 17
    assert p[:5] == bytes([9, 6, 0, 0, 10])
    assert struct.unpack_from("<h", p, 7)[0] == 20000
    assert struct.unpack_from("<h", p, 13)[0] == -4550


def test_virtual_sticks():
    c = commands.virtual_sticks(512, 600, 1023, 0)
    assert c.src == Module.RC and not c.ack and c.header_flags == 0
    assert c.payload == bytes([11, 2, 0, 0]) + struct.pack(
        "<6hH", 512, 600, 1023, 0, 512, 512, 0x3C
    )


def test_stick_directions_match_real_rc():
    from openfimi.sticks import Sticks

    # Measured on an RCX6E: forward/up read low, right reads high.
    assert Sticks(pitch=1.0).raw()[1] == 0
    assert Sticks(throttle=1.0).raw()[2] == 0
    assert Sticks(roll=1.0).raw()[0] == 1023
    assert Sticks(yaw=-1.0).raw()[3] == 0
    assert Sticks().raw() == (512, 512, 512, 512)


def test_real_rc_stick_frame_decodes():
    from openfimi import telemetry
    from openfimi.framing import InnerDecoder, OuterDecoder

    # Captured from an RCX6E through the Android bridge: idle sticks, then photo pressed.
    wire = bytes.fromhex(
        "ae71020021fe8408000d070000d30d2dedb0d5424b0b020000000200020002000200020002bc67"
    )
    ((stype, body),) = OuterDecoder().feed(wire)
    (frame,) = InnerDecoder().feed(body)
    m = telemetry.decode(frame)
    assert isinstance(m, telemetry.RcSticks)
    assert (m.roll, m.pitch, m.throttle, m.yaw, m.wheel) == (512,) * 5
    assert not (m.rth_pressed or m.photo_pressed or m.video_pressed)
    photo = telemetry.RcSticks.decode(bytes(4 + 12)[4:] + bytes((0x34, 0x00)))
    assert photo.photo_pressed and not photo.video_pressed


def test_flags():
    assert commands.takeoff().header_flags == 1
    assert commands.sync_time().header_flags == 0
    assert commands.set_camera_clock().header_flags == 1


def test_activation_vector():
    pytest.importorskip("cryptography")
    c = commands.activate_device(b"ABCDEFGHIJKLMNOP", 2)
    # Reference block from javax.crypto with the app's key/plaintext construction.
    assert c.payload == bytes([1, 23, 0, 0]) + bytes.fromhex("25917f736987fae0af6f6374be447911")
