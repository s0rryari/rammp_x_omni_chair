#!/usr/bin/env python3
"""Fake RAMMP joystick: publishes rammp::XYTwist over RTPS from the keyboard or mouse.

W/S = forward/back, A/D = left/right, Q/E = twist, Space = joystick button,
or drag inside the circle. Release everything and it sends zeros.
"""
import argparse
import math
import os
import sys
import threading
import time
import tkinter as tk

HMI_SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                           "pace-hmi-fw", "scripts")
sys.path.insert(0, os.path.abspath(HMI_SCRIPTS))
import rammp_rtps as spec  # noqa: E402
import rtps_host           # noqa: E402
import rtps_net            # noqa: E402

# Must match kJoystickXYTwist in this repo's external/rammp-rtps.
TOPIC = "rt/rammp/joystick/xy_twist"
TYPE = "rammp_peripheral_interfaces::msg::dds_::XYTwist_"


class JoystickSim(rtps_host.RtpsHostHarness):
    def __init__(self, args):
        super().__init__(args)
        self.lock = threading.Lock()
        self.stick = (0.0, 0.0, 0.0, 0)
        self.sent = 0

    def set_stick(self, x, y, twist, buttons):
        with self.lock:
            self.stick = (x, y, twist, buttons)

    def publish_now(self):  # run() calls this every publish_interval
        with self.lock:
            x, y, twist, buttons = self.stick
        writer = self.local_writers[0]
        message = self.build_data_message(
            writer, spec.encode(spec.MSG_XY_TWIST, x, y, twist, buttons))
        for target in self._build_user_targets(writer):
            self.send_user_datagram(message, target)
            self.sent += 1


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--peer", default="192.168.4.1", help="the P4's address")
    p.add_argument("--advertised-address", default=None,
                   help="this laptop's address on the P4 link (auto-detected)")
    p.add_argument("--rate", type=float, default=50.0, help="messages per second")
    p.add_argument("--speed", type=float, default=0.5, help="keyboard deflection, 0..1")
    cli = p.parse_args()

    advertised = cli.advertised_address or rtps_net.source_address_for(cli.peer)
    if not advertised:
        print(f"No route to {cli.peer}: is the cable in and the laptop on DHCP?")
        return 1
    print(f"Publishing {TOPIC} to {cli.peer} from {advertised}")

    sim = JoystickSim(argparse.Namespace(
        node_name="joystick_sim", domain_id=0, participant_id=14,
        bind_address=advertised, advertised_address=advertised,
        multicast_interface=None, multicast_group="239.255.0.1", enclave="/",
        subscribe_topic=[], subscribe_type_name=None,
        publish_topic=TOPIC, type_name=TYPE, reliable=False,
        publish_value=0, publish_interval=1.0 / cli.rate,
        echo_received=False, announce_period=1.0, duration=0.0, trace_packets=False,
        peer=[cli.peer], peer_participant_ids=rtps_host.parse_participant_id_range("0-3"),
    ))
    threading.Thread(target=sim.run, daemon=True).start()

    # --- window: a drag pad plus keyboard ---------------------------------
    root = tk.Tk()
    root.title("RAMMP joystick sim")
    size, r = 240, 100
    c = size / 2
    pad = tk.Canvas(root, width=size, height=size, bg="white")
    pad.pack()
    pad.create_oval(c - r, c - r, c + r, c + r)
    knob = pad.create_oval(c - 8, c - 8, c + 8, c + 8, fill="black")
    status = tk.Label(root, font=("Consolas", 11))
    status.pack()
    tk.Label(root, text="W/S fwd/back   A/D left/right   Q/E twist   Space button\n"
                        "or drag inside the circle").pack()

    keys = set()
    mouse = [None]  # (x, y) while dragging
    root.bind("<KeyPress>", lambda e: keys.add(e.keysym.lower()))
    root.bind("<KeyRelease>", lambda e: keys.discard(e.keysym.lower()))

    def drag(e):
        mouse[0] = ((e.x - c) / r, (c - e.y) / r)

    pad.bind("<Button-1>", drag)
    pad.bind("<B1-Motion>", drag)
    pad.bind("<ButtonRelease-1>", lambda e: mouse.__setitem__(0, None))

    def tick():
        if mouse[0]:
            x, y = mouse[0]
        else:
            x = cli.speed * (("d" in keys) - ("a" in keys))  # + = right
            y = cli.speed * (("w" in keys) - ("s" in keys))  # + = forward
        twist = cli.speed * (("e" in keys) - ("q" in keys))  # + = clockwise
        n = math.hypot(x, y)
        if n > 1:  # XYTwist keeps x/y inside the unit circle
            x, y = x / n, y / n
        buttons = 1 if "space" in keys else 0
        sim.set_stick(x, y, twist, buttons)
        pad.coords(knob, c + x * r - 8, c - y * r - 8, c + x * r + 8, c - y * r + 8)
        status.config(text=f"x={x:+.2f} y={y:+.2f} twist={twist:+.2f} "
                           f"button={buttons}  sent={sim.sent}")
        root.after(20, tick)

    def close():
        sim.set_stick(0.0, 0.0, 0.0, 0)  # leave the chair at rest
        root.after(150, lambda: (sim.stop(), root.destroy()))

    root.protocol("WM_DELETE_WINDOW", close)
    tick()
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
