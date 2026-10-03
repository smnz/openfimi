"""Take off, point the camera down, take one photo, land.

    python examples/survey_photo.py tcp://127.0.0.1:10052     # against `openfimi sim`
"""

import sys
import time

from openfimi import Drone, transport


def main(url: str) -> None:
    with Drone(transport.from_url(url)) as d:
        if not d.wait_for_telemetry(10):
            sys.exit("no telemetry: is the RC connected and the aircraft on?")
        print("before:", d.state.summary())

        d.takeoff(check=True)
        if not d.wait_until(lambda s: s.sport.height_m > 1.0, timeout=20):
            d.land()
            sys.exit("did not climb; landing")

        d.gimbal_pitch(-90)
        time.sleep(2)
        print("photo:", d.take_photo())

        d.land(check=True)
        d.wait_until(lambda s: not s.heart.flying, timeout=60)
        print("after:", d.state.summary())


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "tcp://127.0.0.1:10052")
