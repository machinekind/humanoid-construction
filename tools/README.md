# tools

## dm_motor_setup.py — Damiao motor commissioning over the RoboParty roboto_usb2can board

Talks gs_usb/candleLight directly through libusb, so it works on macOS (no SocketCAN needed) as well as Linux.
Classic CAN 2.0 at 1 Mbit/s, master id = slave id + 0x10, matching RoboParty's `roboparty_deploy` config.

```bash
brew install libusb                       # Linux: apt install libusb-1.0-0
python3 -m venv ~/.venv/dm && source ~/.venv/dm/bin/activate
pip install gs_usb pyusb
python tools/dm_motor_setup.py --help
```

Per-motor bench flow (new motors ship as id 1, so always one motor on the bus at a time):

```bash
python tools/dm_motor_setup.py next          # assigns the next unassigned joint id, saves to flash, logs it
# power-cycle the motor
python tools/dm_motor_setup.py check --id N  # verifies the saved config against the RoboParty target
```

`commissioned.json` (git-ignored, local working state) is the log `next` uses to know which joint comes next.

Board socket → adapter USB serial, as wired on 2026-10-09 (needed for the udev `canX` rules on the robot controller):

| socket | adapter serial suffix | RoboParty bus |
|---|---|---|
| 1 | `…006D0052` | can0 — left leg |
| 2 | `…00650053` | can1 — right leg + waist |
| 3 / 4 | `…0038006D`, `…0046001E` | can2 / can3 — not yet assigned |

Read-only: `scan`, `info`, `check`, `log`. Writes flash: `next`, `set-id`, `fix-master`, `zero`. Energises the motor:
`test`, `move`, `zero`. See the docstring at the top of the script for protocol details and firmware quirks.
