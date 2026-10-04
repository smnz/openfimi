"""Command-line interface: ``openfimi <command> ...``."""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
from pathlib import Path

from . import commands, telemetry
from .drone import Drone, PreflightError
from .framing import InnerDecoder, OuterDecoder, StreamType
from .link import AckTimeout
from .mission import mission_from_dict
from .transport import RecordingTransport, from_url
from .transport.capture import RX, read_capture
from .video import Depacketizer, FfmpegSink

DEFAULT_URL = os.environ.get("OPENFIMI_URL", "usb")


def _transport(args):
    t = from_url(args.url)
    if getattr(args, "record", None):
        t = RecordingTransport(t, open(args.record, "wb"))
    return t


def _confirm(args, what: str) -> None:
    if args.yes:
        return
    ans = input(f"About to {what}. Props clear and area safe? Type 'yes': ")
    if ans.strip().lower() != "yes":
        sys.exit("aborted")


# -- commands ----------------------------------------------------------------


def cmd_doctor(args) -> int:
    """Check whether this machine can act as the phone (USB gadget)."""
    from .transport.aoa import CONFIGFS, GadgetConfig

    udcs = GadgetConfig.available_udcs()
    print(f"USB device controllers (/sys/class/udc): {udcs or 'NONE'}")
    print(f"configfs usb_gadget: {'present' if CONFIGFS.exists() else 'missing'}")
    print(f"running as root: {os.geteuid() == 0}")
    if not udcs:
        print(
            "\nThis machine cannot be a USB device, so it cannot replace the phone.\n"
            "Use a Pi Zero 2 W / Pi 4 / Pi 5 with dtoverlay=dwc2 as the gadget and\n"
            "run `openfimi bridge` there, then use -u tcp://<pi> from here."
        )
        return 1
    return 0


def cmd_bridge(args) -> int:
    from .bridge import Bridge

    host, _, port = args.listen.rpartition(":")
    upstream = _transport(args)
    Bridge(upstream, host or "0.0.0.0", int(port)).serve_forever()
    return 0


def cmd_monitor(args) -> int:
    with Drone(_transport(args), init_camera=not args.quiet_connect) as d:
        if args.raw:
            d.link.on_frame(lambda f: print(f"{time.time():.3f} {f.describe()}", flush=True))
        if not d.wait_for_telemetry(args.wait):
            print(
                f"no FC telemetry after {args.wait}s (link stats: {d.link.stats})", file=sys.stderr
            )
        try:
            while not d.link.closed.is_set():
                if not args.raw:
                    print(json.dumps(d.state.summary()), flush=True)
                time.sleep(args.interval)
        except KeyboardInterrupt:
            pass
        print(f"\nlink: {d.link.stats}", file=sys.stderr)
    return 0


def cmd_video(args) -> int:
    # Purely passive by default: video flows without any command, so a viewer
    # never interferes with another client driving the aircraft.
    with Drone(_transport(args), init_camera=args.fpv_config) as d:
        if args.output:
            sink = (
                FfmpegSink.ffmpeg(args.output, args.codec)
                if args.output.endswith((".mp4", ".mkv", ".mov"))
                else None
            )
            if sink is None:
                fp = sys.stdout.buffer if args.output == "-" else open(args.output, "wb")

                def write(p, fp=fp):
                    if p.is_video:
                        try:
                            fp.write(p.data)
                            fp.flush()
                        except BrokenPipeError:
                            d.link.closed.set()  # the reader went away: stop

                d.on_video(write)
        else:
            sink = FfmpegSink.ffplay(args.codec)
        if sink is not None:
            d.on_video(sink)
        try:
            while not d.link.closed.is_set():
                time.sleep(1)
                if args.verbose:
                    print(f"video packets: {d.link.stats.video_packets}", file=sys.stderr)
        except KeyboardInterrupt:
            pass
    return 0


_SIMPLE = {
    "takeoff": (commands.takeoff, "TAKE OFF"),
    "land": (commands.land, "LAND"),
    "rth": (commands.return_home, "RETURN HOME"),
    "cancel-takeoff": (commands.cancel_takeoff, None),
    "cancel-land": (commands.cancel_land, None),
    "cancel-rth": (commands.cancel_return_home, None),
    "photo": (commands.take_photo, None),
    "record-start": (commands.start_recording, None),
    "record-stop": (commands.stop_recording, None),
    "mission-start": (commands.mission_start, "START THE UPLOADED MISSION"),
    "mission-stop": (commands.mission_stop, None),
}


def _print_reply(r, cmd=None) -> None:
    if r is None:
        print(f"{cmd.name if cmd else 'command'}: sent (no ack expected)")
    else:
        print(
            f"{r.command.name}: code={r.code} {'OK' if r.ok else 'REJECTED?'}  "
            f"[{r.frame.describe()}]"
        )


def _say(msg: str) -> None:
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


def _wait_args(sp) -> None:
    sp.add_argument(
        "--wait-gps",
        type=float,
        nargs="?",
        const=600.0,
        metavar="SECONDS",
        help="wait (default up to 600 s) for GPS, home point and take-off clearance",
    )
    sp.add_argument(
        "--min-sats", type=int, default=10, help="satellites required before take-off (default 10)"
    )
    sp.add_argument(
        "--settle",
        type=float,
        default=5.0,
        metavar="SECONDS",
        help="with --wait-gps: require the aircraft level and still on the ground this long "
        "(default 5; 0 = off)",
    )


def cmd_send(args) -> int:
    if args.action == "gimbal":
        if args.value is None:
            sys.exit("gimbal needs a pitch in degrees, e.g. `send gimbal -- -45`")
        cmd = commands.gimbal_pitch(float(args.value))
    elif args.action == "raw":
        mod, _, hexp = (args.value or "").partition(":")
        cmd = commands.raw(int(mod), bytes.fromhex(hexp))
    else:
        build, danger = _SIMPLE[args.action]
        if danger:
            _confirm(args, danger)
        cmd = build()
    with Drone(_transport(args), init_camera=False) as d:
        d.wait_for_telemetry(args.wait)
        if args.action == "takeoff" and args.wait_gps is not None:
            try:
                d.wait_until_ready(
                    args.wait_gps, min_satellites=args.min_sats, settle_s=args.settle, on_event=_say
                )
            except PreflightError as e:
                sys.exit(f"not ready: {e}")
        try:
            r = d.send(cmd)
        except AckTimeout as e:
            print(f"timeout: {e}", file=sys.stderr)
            return 2
        _print_reply(r, cmd)
    return 0


def _load_route(args):
    from .mission import mission_from_fimi_db

    if args.fimi_db:
        if not args.route:
            sys.exit("--fimi-db needs --route NAME (or _id)")
        route = int(args.route) if args.route.isdigit() else args.route
        m = mission_from_fimi_db(args.fimi_db, route)
    elif args.file:
        m = mission_from_dict(json.loads(Path(args.file).read_text()))
    else:
        sys.exit("give a route JSON file or --fimi-db DB --route NAME")
    from .mission import GimbalMode

    if args.pitch is not None:
        for w in m.waypoints:
            w.gimbal_pitch_deg = args.pitch
            if args.gimbal_mode is None:
                w.gimbal_mode = GimbalMode.BEFORE_ARRIVAL
    if args.gimbal_mode is not None:
        mode = {
            "none": GimbalMode.NONE,
            "before": GimbalMode.BEFORE_ARRIVAL,
            "arrival": GimbalMode.ON_ARRIVAL,
        }[args.gimbal_mode]
        for w in m.waypoints:
            w.gimbal_mode = mode
    m.validate()
    return m


def cmd_mission(args) -> int:
    with Drone(_transport(args), init_camera=False) as d:
        d.wait_for_telemetry(args.wait)
        if args.op == "upload":
            m = _load_route(args)
            d.upload_mission(m, progress=lambda i, n: print(f"\r{i}/{n}", end="", flush=True))
            print("\nuploaded", len(m.waypoints), "waypoints")
        elif args.op == "fly":
            m = _load_route(args)
            gim = [
                f"wp{i} {w.gimbal_pitch_deg:g} deg {w.gimbal_mode.name.lower()}"
                for i, w in enumerate(m.waypoints)
                if w.gimbal_mode
            ]
            print(
                f"route: {len(m.waypoints)} waypoints, finish={m.finish!r}, "
                f"gimbal: {', '.join(gim) if gim and args.gimbal != 'off' else 'untouched'}"
            )
            d.wait_until(lambda s: s.battery and s.signal and s.errors, 5)
            problems = d.preflight(min_satellites=args.min_sats)
            if problems and not d.state.flying:
                if args.wait_gps is None:
                    sys.exit("not ready: " + "; ".join(problems))
                print("not ready yet (will wait): " + "; ".join(problems))
            _confirm(args, f"FLY THE {len(m.waypoints)}-WAYPOINT ROUTE")
            try:
                res = d.fly_route(
                    m,
                    wait_ready=args.wait_gps or 0.0,
                    min_satellites=args.min_sats,
                    settle_s=args.settle,
                    follow_gimbal=False if args.gimbal == "off" else None,
                    gimbal_lead_s=args.lead,
                    on_event=_say,
                )
            except PreflightError as e:
                sys.exit(f"not ready: {e}")
            print(res)
        elif args.op == "read":
            for p in d.read_mission():
                print(json.dumps(p.as_dict()))
        elif args.op == "start":
            _confirm(args, "START THE UPLOADED MISSION")
            _print_reply(d.start_mission())
        elif args.op == "stop":
            _print_reply(d.stop_mission())
    return 0


def cmd_decode(args) -> int:
    """Decode a capture (.ofcap) or a raw byte dump offline."""
    outer, inner, video = OuterDecoder(), InnerDecoder(), Depacketizer()
    vid_out = open(args.video, "wb") if args.video else None
    counts: dict = {}
    with open(args.file, "rb") as fp:
        head = fp.read(8)
        fp.seek(0)
        if head.startswith(b"OFCAP"):
            chunks = ((t, d) for direction, t, d in read_capture(fp) if direction == RX)
        else:
            chunks = ((0.0, fp.read()),)
        for t, data in chunks:
            for stype, body in outer.feed(data):
                counts[stype] = counts.get(stype, 0) + 1
                if stype == StreamType.FMLINK:
                    for f in inner.feed(body):
                        msg = None
                        try:
                            msg = telemetry.decode(f)
                        except ValueError:
                            pass
                        if not args.quiet:
                            line = f"{t:.3f} {f.describe()}"
                            if msg is not None:
                                line += f"  {type(msg).__name__} {msg.as_dict()}"
                            print(line)
                elif stype == StreamType.VIDEO and vid_out:
                    for p in video.feed(body):
                        if p.is_video:
                            vid_out.write(p.data)
    print(
        f"records by stream type: {counts}; outer checksum errors: "
        f"{outer.bad_checksums}; inner CRC errors: {inner.bad_crc}",
        file=sys.stderr,
    )
    return 0


def cmd_sim(args) -> int:
    from .sim import serve

    host, _, port = args.listen.rpartition(":")
    serve(host or "127.0.0.1", int(port))
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="openfimi", description=__doc__)
    p.add_argument("-v", "--verbose", action="store_true")
    sub = p.add_subparsers(dest="cmd", required=True)

    def link_args(sp):
        sp.add_argument(
            "-u",
            "--url",
            default=DEFAULT_URL,
            help="transport: usb | tcp://host[:port] | udp | tty:///dev/x | "
            "replay://file (default $OPENFIMI_URL or usb)",
        )
        sp.add_argument("--record", metavar="FILE", help="record the session to a capture file")
        sp.add_argument("--wait", type=float, default=10.0, help="seconds to wait for telemetry")

    sp = sub.add_parser("doctor", help="check USB gadget capability")
    sp.set_defaults(fn=cmd_doctor)

    sp = sub.add_parser("bridge", help="relay the RC link over TCP (run on the gadget board)")
    link_args(sp)
    sp.add_argument("--listen", default="0.0.0.0:10052")
    sp.set_defaults(fn=cmd_bridge)

    sp = sub.add_parser("monitor", help="print live telemetry")
    link_args(sp)
    sp.add_argument("--raw", action="store_true", help="print every frame")
    sp.add_argument("--interval", type=float, default=0.5)
    sp.add_argument(
        "--quiet-connect", action="store_true", help="send nothing on connect (pure listen)"
    )
    sp.set_defaults(fn=cmd_monitor)

    sp = sub.add_parser("video", help="show or save the FPV stream")
    link_args(sp)
    sp.add_argument(
        "-o", "--output", help="'-' for raw stream on stdout, .h265/.h264 raw file, or .mp4/.mkv"
    )
    sp.add_argument("--codec", default="hevc", choices=["hevc", "h264"])
    sp.add_argument(
        "--fpv-config", action="store_true", help="send the app's FPV config command on connect"
    )
    sp.set_defaults(fn=cmd_video)

    sp = sub.add_parser("send", help="send one command")
    link_args(sp)
    sp.add_argument("action", choices=sorted([*_SIMPLE, "gimbal", "raw"]))
    sp.add_argument("value", nargs="?", help="gimbal: pitch deg; raw: MODULE:HEXPAYLOAD")
    sp.add_argument("-y", "--yes", action="store_true", help="skip the safety prompt")
    _wait_args(sp)
    sp.set_defaults(fn=cmd_send)

    sp = sub.add_parser("mission", help="upload/read/start/stop a waypoint route")
    link_args(sp)
    sp.add_argument(
        "op",
        choices=["fly", "upload", "read", "start", "stop"],
        help="fly = take off, upload, verify, start, follow gimbal, land",
    )
    sp.add_argument("file", nargs="?", help="route JSON (fly/upload)")
    sp.add_argument("--fimi-db", help="FIMI app database to read the route from")
    sp.add_argument("--route", help="route name or _id in --fimi-db")
    sp.add_argument(
        "--pitch",
        type=float,
        help="every waypoint's gimbal pitch, degrees (-90 = down); implies --gimbal-mode before",
    )
    sp.add_argument(
        "--gimbal-mode",
        choices=["none", "before", "arrival"],
        help="set every waypoint's gimbal mode (default: as stored per waypoint)",
    )
    sp.add_argument(
        "--gimbal",
        default="auto",
        choices=["auto", "off"],
        help="off = ignore all gimbal settings for this flight",
    )
    sp.add_argument(
        "--lead",
        type=float,
        default=15.0,
        help="'before' mode: seconds before arrival (default 15)",
    )
    _wait_args(sp)
    sp.add_argument("-y", "--yes", action="store_true")
    sp.set_defaults(fn=cmd_mission)

    sp = sub.add_parser("sim", help="run a simulated aircraft on a TCP port")
    sp.add_argument("--listen", default="127.0.0.1:10052")
    sp.set_defaults(fn=cmd_sim)

    sp = sub.add_parser("decode", help="decode a capture file offline")
    sp.add_argument("file")
    sp.add_argument("--video", help="also write the video elementary stream here")
    sp.add_argument("-q", "--quiet", action="store_true")
    sp.set_defaults(fn=cmd_decode)

    args = p.parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
