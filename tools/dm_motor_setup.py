#!/usr/bin/env python3
"""Damiao (DM) motor commissioning over the RoboParty roboto_usb2can board, from macOS/Linux.

Talks gs_usb (candleLight) directly through libusb, so no SocketCAN is needed.
Classic CAN 2.0 at 1 Mbit/s, which is what the RoboParty Roboto config uses.

Setup (once):
    brew install libusb                      # macOS; Linux: apt install libusb-1.0-0
    python3 -m venv ~/.venv/dm && source ~/.venv/dm/bin/activate
    pip install gs_usb pyusb

Commands (all take --adapter <usb-serial-suffix>; omit it if only one adapter has a live bus):
    scan                    list adapters, which buses have powered nodes, which motor IDs answer
    info    --id N          dump the full register map + one status frame (the UART "debug banner" over CAN)
    check   --id N          compare the motor against the RoboParty target config for that joint
    set-id  --id OLD --new NEW   re-ID a motor (slave=NEW, master=NEW+0x10) and save to flash.
                            Do this with the motor ALONE on the bus: every new DM motor ships as ID 1.
    fix-master --id N       write master = N + 0x10 and save (only this, nothing else)
    test    --id N          3 s damping-mode enable (kp 0, kd 1): proves encoder/commutation; motor is energised
    zero    --id N          enable in damping mode, show live position, Enter saves zero, disable
    move    --id N          functional test: enable, MIT step of --delta rad (default 0.3) at low gains, return, disable
    next                    sequential commissioning: the ONE motor on the live bus (factory id 1) gets the next
                            unassigned RoboParty id, master = id+0x10, saved to flash, and is recorded in
                            commissioned.json next to this script. Plug motor, run `next`, power-cycle, `check`, unplug, repeat.
    log                     show commissioned.json

Read-only commands: scan, info, check, log.  Writing commands: set-id, fix-master, next, zero.  Energising: test, zero, move.

Protocol (from roboparty_motors dm_motor_driver.cpp and the DM docs):
    0x7FF frames, data[0..1] = target slave id LE:  0x33 read reg, 0x55 write reg, 0xAA save flash, 0xCC refresh status
    <slave id> frames, data[7]: 0xFC enable, 0xFD disable, 0xFE save zero, 0xFB clear error; otherwise MIT command
    feedback arrives on <master id>:  [id|err<<4, pos16, vel12, tau12, mos_temp, coil_temp]
"""
import argparse
import datetime
import json
import os
import select
import struct
import sys
import time

import usb.core
import usb.util
from gs_usb.gs_usb import GsUsb, GS_CAN_MODE_NORMAL, GS_CAN_MODE_HW_TIMESTAMP
from gs_usb.gs_usb_frame import GsUsbFrame, GS_USB_NONE_ECHO_ID
from gs_usb.constants import CAN_ERR_FLAG

BITRATE = 1_000_000
LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "commissioned.json")
MASTER_OFFSET = 0x10          # RoboParty master_id_offset: 16

# RoboParty Roboto Origin joint table: motor id -> (joint, bus, model).  Order = URDF order, urdf2motor is identity.
MODEL_NAME = {0: "DM4340P-48V", 1: "DM10010L-48V"}
JOINTS = {}
for _i, (_j, _bus, _m) in enumerate([
    ("left_thigh_yaw", "can0 left leg", 1), ("left_thigh_roll", "can0 left leg", 1), ("left_thigh_pitch", "can0 left leg", 1),
    ("left_knee", "can0 left leg", 1), ("left_ankle_pitch", "can0 left leg", 0), ("left_ankle_roll", "can0 left leg", 0),
    ("right_thigh_yaw", "can1 right leg+waist", 1), ("right_thigh_roll", "can1 right leg+waist", 1), ("right_thigh_pitch", "can1 right leg+waist", 1),
    ("right_knee", "can1 right leg+waist", 1), ("right_ankle_pitch", "can1 right leg+waist", 0), ("right_ankle_roll", "can1 right leg+waist", 0),
    ("torso", "can1 right leg+waist", 1),
    ("left_arm_pitch", "can2 left arm", 0), ("left_arm_roll", "can2 left arm", 0), ("left_arm_yaw", "can2 left arm", 0),
    ("left_elbow_pitch", "can2 left arm", 0), ("left_elbow_yaw", "can2 left arm", 0),
    ("right_arm_pitch", "can3 right arm", 0), ("right_arm_roll", "can3 right arm", 0), ("right_arm_yaw", "can3 right arm", 0),
    ("right_elbow_pitch", "can3 right arm", 0), ("right_elbow_yaw", "can3 right arm", 0),
]):
    JOINTS[_i + 1] = (_j, _bus, _m)

# Firmware family prefix per model (DM docs: first two digits of the firmware version identify the motor series)
FW_PREFIX = {0: "61", 1: "67"}

# (reg, name, type f=float32/u=uint32, comment)
REGS = [
    (0, "UV_Value", "f", "V under-voltage"), (1, "KT_Value", "f", "Nm/A"), (2, "OT_Value", "f", "degC over-temp"),
    (3, "OC_Value", "f", "A over-current"), (4, "ACC", "f", "rad/s^2"), (5, "DEC", "f", "rad/s^2"), (6, "MAX_SPD", "f", "rad/s"),
    (7, "MST_ID", "u", "master/feedback id"), (8, "ESC_ID", "u", "slave/command id"), (9, "TIMEOUT", "u", "ms"),
    (10, "CTRL_MODE", "u", "1 MIT 2 pos-spd 3 spd 4 hybrid"), (11, "Damp", "f", "viscous friction"), (12, "Inertia", "f", "kg m^2"),
    (13, "hw_ver", "u", ""), (14, "sw_ver", "u", ""), (15, "SN", "u", ""), (16, "NPP", "u", "pole pairs"),
    (17, "Rs", "f", "ohm"), (18, "Ls", "f", "H"), (19, "Flux", "f", "Wb"), (20, "Gr", "f", "gear ratio"),
    (21, "PMAX", "f", "rad"), (22, "VMAX", "f", "rad/s"), (23, "TMAX", "f", "Nm"), (24, "I_BW", "f", "Hz"),
    (25, "KP_ASR", "f", ""), (26, "KI_ASR", "f", ""), (27, "KP_APR", "f", ""), (28, "KI_APR", "f", ""),
    (29, "OV_Value", "f", "V over-voltage"), (30, "GREF", "f", ""), (31, "Deta", "f", ""), (32, "V_BW", "f", ""),
    (33, "IQ_c1", "f", ""), (34, "VL_c1", "f", ""), (35, "can_br", "u", "0:125k 1:200k 2:250k 3:500k 4:1M"), (36, "sub_ver", "u", ""),
    (50, "u_off", "f", ""), (51, "v_off", "f", ""), (52, "k1", "f", ""), (53, "k2", "f", ""),
    (54, "m_off", "f", "rad mechanical zero offset"), (55, "dir", "f", ""), (80, "p_m", "f", "rad motor-side pos"), (81, "xout", "f", "rad output pos"),
]
REG_TYPE = {r: t for r, _, t, _ in REGS}
ERR = {0: "disabled", 1: "enabled", 8: "over-voltage", 9: "under-voltage", 10: "over-current",
       11: "MOS over-temp", 12: "coil over-temp", 13: "comm lost", 14: "overload"}


def state_text(mid, err):
    """Feedback byte 0 = (err << 4) | id, but this firmware puts the full id low byte there, so for ids >= 16
    bit 4 of the id lands in the state nibble: enabled/disabled is unreadable, faults (>= 8) still show."""
    if err >= 8:
        return ERR.get(err, f"fault {err}")
    if mid >= 16:
        return "state n/a (id>=16 masks the enable bit)"
    return ERR.get(err, "?")


class Bus:
    """One roboto_usb2can channel."""

    def __init__(self, dev):
        self.dev = dev
        dev.gs_usb.is_kernel_driver_active = lambda i: False   # macOS: detach needs root and is not needed
        dev.gs_usb.reset = lambda: None                        # gs_usb.start() resets the USB device, which makes the OS re-enumerate it
        try:
            dev.stop()
        except Exception:
            pass
        dev.set_bitrate(BITRATE)
        dev.start(GS_CAN_MODE_NORMAL | GS_CAN_MODE_HW_TIMESTAMP)

    def close(self):
        try:
            self.dev.stop()
        except Exception:
            pass
        usb.util.dispose_resources(self.dev.gs_usb)   # release the claimed interface so the device can be reopened

    def send(self, can_id, data):
        self.dev.send(GsUsbFrame(can_id=can_id, data=list(data) + [0] * (8 - len(data))))

    def recv(self, seconds, pred=lambda f: True):
        """First non-echo, non-error frame matching pred within `seconds`, else None."""
        end = time.time() + seconds
        while time.time() < end:
            f = GsUsbFrame()
            try:
                if not self.dev.read(f, 20):
                    continue
            except usb.core.USBTimeoutError:
                continue
            if f.echo_id != GS_USB_NONE_ECHO_ID or f.can_id & CAN_ERR_FLAG:
                continue
            if pred(f):
                return f
        return None

    def drain(self):
        """Discard buffered frames (a control loop leaves many feedback frames queued)."""
        while self.recv(0.03) is not None:
            pass

    def send_acked(self, can_id, data, seconds=0.05):
        """Send and report whether the TX echo returned (== another node ACKed the frame)."""
        self.send(can_id, data)
        end = time.time() + seconds
        while time.time() < end:
            f = GsUsbFrame()
            try:
                if not self.dev.read(f, 20):
                    continue
            except usb.core.USBTimeoutError:
                continue
            if f.echo_id != GS_USB_NONE_ECHO_ID:
                return True
        return False

    # ---- DM register protocol (0x7FF) ----
    def read_reg(self, mid, rid, timeout=0.2):
        self.send(0x7FF, [mid & 0xFF, mid >> 8, 0x33, rid, 0xFF, 0xFF, 0xFF, 0xFF])
        f = self.recv(timeout, lambda f: f.can_dlc == 8 and f.data[2] == 0x33 and f.data[3] == rid)
        if f is None:
            return None
        raw = bytes(f.data[4:8])
        return struct.unpack("<f" if REG_TYPE.get(rid) == "f" else "<I", raw)[0]

    def write_reg(self, mid, rid, value):
        raw = struct.pack("<f" if REG_TYPE.get(rid) == "f" else "<I", value)
        self.send(0x7FF, [mid & 0xFF, mid >> 8, 0x55, rid] + list(raw))
        f = self.recv(0.3, lambda f: f.can_dlc == 8 and f.data[2] == 0x55 and f.data[3] == rid)
        return f is not None

    def save_flash(self, mid):
        self.send(0x7FF, [mid & 0xFF, mid >> 8, 0xAA, 0x01, 0xFF, 0xFF, 0xFF, 0xFF])
        return self.recv(0.5, lambda f: f.data[2] == 0xAA) is not None

    # ---- motor control frames (slave id) ----
    def ctrl(self, mid, last):
        self.send(mid, [0xFF] * 7 + [last])

    def enable(self, mid):  self.ctrl(mid, 0xFC)
    def disable(self, mid): self.ctrl(mid, 0xFD)
    def save_zero(self, mid): self.ctrl(mid, 0xFE)
    def clear_error(self, mid): self.ctrl(mid, 0xFB)

    def mit(self, mid, lim, p=0.0, v=0.0, kp=0.0, kd=0.0, t=0.0):
        pmax, vmax, tmax = lim
        def q(x, lo, hi, bits):
            x = min(max(x, lo), hi)
            return int((x - lo) / (hi - lo) * ((1 << bits) - 1))
        p_i, v_i = q(p, -pmax, pmax, 16), q(v, -vmax, vmax, 12)
        kp_i, kd_i, t_i = q(kp, 0, 500, 12), q(kd, 0, 5, 12), q(t, -tmax, tmax, 12)
        self.send(mid, [p_i >> 8, p_i & 0xFF, v_i >> 4, ((v_i & 0xF) << 4) | (kp_i >> 8), kp_i & 0xFF,
                        kd_i >> 4, ((kd_i & 0xF) << 4) | (t_i >> 8), t_i & 0xFF])

    def status(self, mid, lim, master=None, timeout=0.3, refresh=True):
        """Request (0xCC) and decode one feedback frame. Returns dict or None."""
        if refresh:
            self.drain()
            self.send(0x7FF, [mid & 0xFF, mid >> 8, 0xCC, 0, 0, 0, 0, 0])

        def is_reg_reply(f):   # register replies look like [id_lo, id_hi, 0x33|0x55|0xAA, reg, ...]; feedback's byte 2 is a position byte
            return f.data[0] == (mid & 0xFF) and f.data[1] == (mid >> 8) and f.data[2] in (0x33, 0x55, 0xAA)
        f = self.recv(timeout, lambda f: f.can_dlc == 8 and (f.data[0] & 0x0F) == (mid & 0x0F)
                      and (f.can_id & 0x7FF) != 0x7FF and not is_reg_reply(f))
        if f is None:
            return None
        d = bytes(f.data[:8])
        pmax, vmax, tmax = lim
        pos = (d[1] << 8 | d[2]) / 65535 * 2 * pmax - pmax
        spd = (d[3] << 4 | d[4] >> 4) / 4095 * 2 * vmax - vmax
        tau = ((d[4] & 0x0F) << 8 | d[5]) / 4095 * 2 * tmax - tmax
        return dict(rx_id=f.can_id & 0x7FF, err=d[0] >> 4, pos=pos, spd=spd, tau=tau, mos=d[6], coil=d[7], raw=d.hex(" "))


def limits(bus, mid):
    lim = tuple(bus.read_reg(mid, r) for r in (21, 22, 23))
    if any(x is None for x in lim):
        sys.exit(f"motor {mid}: no reply reading PMAX/VMAX/TMAX — is it on this bus and powered?")
    return lim


def fmt_u(v):
    raw = struct.pack("<I", v).rstrip(b"\0")
    return f"'{raw.decode()}'" if raw and all(32 <= b < 127 for b in raw) else str(v)


# ---------------------------------------------------------------- adapters
def adapters():
    return sorted(GsUsb.scan(), key=lambda d: d.serial_number)


def pick_adapter(suffix):
    devs = adapters()
    if not devs:
        sys.exit("no gs_usb adapter found (is libusb installed? is the board plugged in?)")
    if suffix:
        m = [d for d in devs if d.serial_number.endswith(suffix)]
        if len(m) != 1:
            sys.exit(f"--adapter {suffix}: {len(m)} matches among {[d.serial_number for d in devs]}")
        return m[0]
    # auto: the single adapter whose bus ACKs a frame
    live = []
    for d in devs:
        b = Bus(d)
        try:
            if b.send_acked(0x7FF, [0, 0, 0x33, 8, 0xFF, 0xFF, 0xFF, 0xFF]):
                live.append(d)
        finally:
            b.close()
    if len(live) != 1:
        sys.exit(f"{len(live)} adapters have a live bus; pass --adapter <serial suffix>. "
                 f"Adapters: {[d.serial_number for d in devs]}")
    return live[0]


# ---------------------------------------------------------------- commands
def cmd_scan(a):
    devs = adapters()
    print(f"{len(devs)} roboto_usb2can channel(s)")
    for d in devs:
        b = Bus(d)
        try:
            found = {}
            acked = 0
            for mid in range(1, a.max_id + 1):
                if b.send_acked(0x7FF, [mid & 0xFF, mid >> 8, 0x33, 8, 0xFF, 0xFF, 0xFF, 0xFF]):
                    acked += 1
                f = b.recv(0.03, lambda f: f.can_dlc == 8 and f.data[2] == 0x33 and f.data[3] == 8)
                if f is not None:
                    found[mid] = struct.unpack("<I", bytes(f.data[4:8]))[0]
            if acked == 0:
                state = "no powered node on bus"
            else:
                state = f"bus live; motor ids answering: {sorted(found) or 'none'}"
            print(f"  sn …{d.serial_number[-8:]}  (usb addr {d.gs_usb.address:2d})  {state}")
            for mid in sorted(found):
                j = JOINTS.get(mid)
                print(f"      id {mid:2d} -> {j[0] if j else 'not in RoboParty table'}")
        finally:
            b.close()


def cmd_info(a):
    b = Bus(pick_adapter(a.adapter))
    try:
        mid = a.id
        print(f"motor id {mid}  ({JOINTS.get(mid, ('?',))[0]})\n--- registers ---")
        vals = {}
        for rid, name, typ, note in REGS:
            v = b.read_reg(mid, rid)
            vals[rid] = v
            if v is None:
                print(f"  {rid:3d} {name:10s} : no reply")
                continue
            s = f"{v:.6g}" if typ == "f" else fmt_u(v)
            if name in ("MST_ID", "ESC_ID"):
                s += f" (0x{v:03X})"
            print(f"  {rid:3d} {name:10s} = {s:<16s} {note}")
        lim = (vals.get(21) or 12.5, vals.get(22) or 25.0, vals.get(23) or 200.0)
        st = b.status(mid, lim)
        print("--- status (0xCC) ---")
        if st is None:
            print("  no status frame")
        else:
            print(f"  rx on 0x{st['rx_id']:03X}  state {st['err']} ({state_text(mid, st['err'])})  "
                  f"pos {st['pos']:+.4f} rad  vel {st['spd']:+.3f} rad/s  tau {st['tau']:+.2f} Nm  "
                  f"MOS {st['mos']} C  coil {st['coil']} C")
    finally:
        b.close()


def cmd_check(a):
    b = Bus(pick_adapter(a.adapter))
    try:
        mid = a.id
        j = JOINTS.get(mid)
        print(f"motor id {mid}: {j[0] if j else 'NOT in RoboParty table'}" + (f"  bus {j[1]}  model {MODEL_NAME[j[2]]}" if j else ""))
        r = {rid: b.read_reg(mid, rid) for rid in (7, 8, 35, 10, 14, 11, 12, 17, 18, 19, 54, 21, 22, 23)}
        if r[8] is None:
            sys.exit("  no reply — motor not on this bus / not powered")
        ok = True

        def row(label, good, val, want):
            nonlocal ok
            ok &= good
            print(f"  [{'OK' if good else '!!'}] {label:34s} {val}  (want {want})")

        row("slave id", r[8] == mid, f"0x{r[8]:03X}", f"0x{mid:03X}")
        row("master id", r[7] == mid + MASTER_OFFSET, f"0x{r[7]:03X}", f"0x{mid + MASTER_OFFSET:03X}")
        row("CAN bitrate code", r[35] == 4, r[35], "4 = 1 Mbps")
        row("control mode", r[10] == 1, r[10], "1 = MIT")
        fw = fmt_u(r[14]).strip("'")
        if j:
            row("firmware family", fw.startswith(FW_PREFIX[j[2]]), fw, f"{FW_PREFIX[j[2]]}xx for {MODEL_NAME[j[2]]}")
        pcal = all(r[k] not in (None, 0.0) for k in (17, 18, 19, 11, 12))
        row("parameter calibration (Rs/Ls/Flux/Damp/J)", pcal, "non-zero" if pcal else "ZERO -> run 参数标定 in DM_Motor_Tool", "non-zero")
        print(f"  [--] mechanical zero offset m_off = {r[54]:+.4f} rad  (0 means zero never saved, or saved at the raw-zero pose)")
        print("  [--] encoder calibration: not readable over CAN; use `test` (motor must run smooth in damping mode)")
        print("RESULT:", "configured" if ok else "needs fixes" + ("; run fix-master" if r[7] != mid + MASTER_OFFSET else ""))
    finally:
        b.close()


def _set_master_and_save(b, mid, master):
    if not b.write_reg(mid, 7, master):
        sys.exit("  write MST_ID: no ack")
    if not b.save_flash(mid):
        sys.exit("  save to flash: no ack")
    time.sleep(0.3)
    got = b.read_reg(mid, 7)
    print(f"  MST_ID now 0x{got:03X}" if got is not None else "  read-back failed")
    return got == master


def cmd_fix_master(a):
    b = Bus(pick_adapter(a.adapter))
    try:
        mid = a.id
        cur = b.read_reg(mid, 7)
        if cur is None:
            sys.exit("no reply from motor")
        print(f"motor {mid}: MST_ID 0x{cur:03X} -> 0x{mid + MASTER_OFFSET:03X}")
        print("  done" if _set_master_and_save(b, mid, mid + MASTER_OFFSET) else "  FAILED")
    finally:
        b.close()


def cmd_set_id(a):
    b = Bus(pick_adapter(a.adapter))
    try:
        old, new = a.id, a.new
        if not 1 <= new <= 0x7FE - MASTER_OFFSET:
            sys.exit("new id out of range")
        if b.read_reg(old, 8) is None:
            sys.exit(f"motor {old}: no reply")
        if new != old and b.read_reg(new, 8) is not None:
            sys.exit(f"REFUSING: something already answers as id {new} on this bus. Re-ID motors one at a time, alone on the bus.")
        j = JOINTS.get(new)
        print(f"motor {old} -> slave 0x{new:03X} / master 0x{new + MASTER_OFFSET:03X}" + (f"   ({j[0]}, {j[1]})" if j else ""))
        if not a.yes and input("  type 'yes' to write and save to flash: ").strip() != "yes":
            sys.exit("aborted")
        if not b.write_reg(old, 8, new):
            sys.exit("  write ESC_ID: no ack")
        time.sleep(0.2)
        if b.read_reg(new, 8) != new:
            sys.exit("  motor does not answer on the new id after write; nothing saved — power-cycle and rescan")
        if _set_master_and_save(b, new, new + MASTER_OFFSET):
            print(f"  saved. verify after power cycle:  {sys.argv[0]} check --id {new}")
        else:
            print("  FAILED to set master; slave id write not yet persisted either (no save) — power-cycle and rescan")
    finally:
        b.close()


def _damping_loop(b, mid, lim, seconds=None, prompt=None):
    """Enable, hold kp=0/kd=1 damping, print live status. Returns ('enter'|'space'|'timeout', last_status)."""
    b.enable(mid)
    time.sleep(0.1)
    st = b.status(mid, lim)
    if st is None or st["err"] >= 8 or (mid < 16 and st["err"] != 1):
        b.disable(mid)
        sys.exit(f"  motor did not enable cleanly: {st}")
    if prompt:
        print(prompt)
    end = time.time() + seconds if seconds else None
    last, key = st, "timeout"
    try:
        if prompt:
            import termios, tty
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        while True:
            b.mit(mid, lim, kp=0.0, kd=1.0)
            st = b.status(mid, lim, refresh=False, timeout=0.05)   # MIT cmd itself elicits feedback
            if st:
                last = st
                print(f"\r  pos {st['pos']:+.4f} rad  vel {st['spd']:+.3f}  tau {st['tau']:+.2f}  state {st['err']} ({state_text(mid, st['err'])})  "
                      f"MOS {st['mos']} C   ", end="", flush=True)
                if st["err"] >= 8:
                    key = "error"
                    break
            if end and time.time() > end:
                break
            if prompt and select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.read(1)
                if c in ("\r", "\n"):
                    key = "enter"; break
                if c == " ":
                    key = "space"; break
                if c == "\x03":
                    raise KeyboardInterrupt
            time.sleep(0.01)
    finally:
        if prompt:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()
    return key, last


def cmd_test(a):
    b = Bus(pick_adapter(a.adapter))
    try:
        mid = a.id
        lim = limits(b, mid)
        print(f"motor {mid}: enabling in damping mode for {a.seconds:.0f} s — it is energised, turn it by hand and watch the numbers")
        key, st = _damping_loop(b, mid, lim, seconds=a.seconds)
        b.disable(mid)
        time.sleep(0.1)
        b.drain()
        off = b.status(mid, lim)
        print(f"  disabled -> state {off['err'] if off else '?'} ({state_text(mid, off['err']) if off else '?'}).  "
              f"{'ERROR during test: ' + ERR.get(st['err'], '?') if key == 'error' else 'OK: no fault, position tracked'}")
    finally:
        b.close()


def cmd_zero(a):
    b = Bus(pick_adapter(a.adapter))
    try:
        mid = a.id
        lim = limits(b, mid)
        j = JOINTS.get(mid)
        print(f"motor {mid} ({j[0] if j else '?'}): damping mode. Move the joint to its mechanical zero.")
        key, st = _damping_loop(b, mid, lim, prompt="  [Enter] = save zero here   [Space] = abort   (Ctrl-C aborts)")
        if key != "enter":
            b.disable(mid)
            print("  aborted, nothing written")
            return
        b.save_zero(mid)
        time.sleep(0.5)
        b.drain()
        st = b.status(mid, lim)
        b.disable(mid)
        time.sleep(0.1)
        if st and abs(st["pos"]) < 0.01:
            print(f"  zero saved: position now {st['pos']:+.4f} rad")
        else:
            print(f"  WARNING: position after save-zero = {st['pos'] if st else '?'} — zero may not have taken; retry")
    finally:
        b.close()


# ---------------------------------------------------------------- sequential flow
def load_log():
    if not os.path.exists(LOG_PATH):
        return []
    with open(LOG_PATH) as f:
        return json.load(f)


def save_log(entries):
    with open(LOG_PATH, "w") as f:
        json.dump(entries, f, indent=2)
        f.write("\n")


def cmd_log(a):
    entries = load_log()
    done = {e["id"] for e in entries}
    print(f"{LOG_PATH}\n  {len(done)}/{len(JOINTS)} motors commissioned")
    for mid, (joint, bus, model) in JOINTS.items():
        e = next((x for x in entries if x["id"] == mid), None)
        mark = f"done {e['date'][:16]}  fw {e.get('fw', '?')}  sn {e.get('sn', '?')}" if e else "----"
        print(f"  id {mid:2d}  {joint:18s} {bus:22s} {MODEL_NAME[model]:13s} {mark}")


def _found_ids(b, max_id=23):
    found = {}
    for mid in range(1, max_id + 1):
        v = b.read_reg(mid, 8, timeout=0.03)
        if v is not None:
            found[mid] = v
    return found


def cmd_next(a):
    entries = load_log()
    done = {e["id"] for e in entries}
    target = a.id or next((i for i in JOINTS if i not in done), None)
    if target is None:
        sys.exit("all 23 joints are already in the log; pass --id to redo one")
    if target in done and not a.force:
        sys.exit(f"id {target} is already logged as commissioned ({[e for e in entries if e['id'] == target][0]['joint']}); pass --force to redo")
    joint, busname, model = JOINTS[target]

    b = Bus(pick_adapter(a.adapter))
    try:
        found = _found_ids(b)
        if len(found) != 1:
            sys.exit(f"expected exactly ONE motor on the live bus, found ids {sorted(found)}. Plug motors in one at a time.")
        cur = next(iter(found))
        sn, fw, sub = b.read_reg(cur, 15), b.read_reg(cur, 14), b.read_reg(cur, 36)
        fw_s = fmt_u(fw).strip("'") if fw is not None else "?"
        print(f"found motor at id {cur}  fw {fw_s}.{fmt_u(sub).strip(chr(39)) if sub is not None else '?'}  sn-reg {fmt_u(sn) if sn is not None else '?'}")
        print(f"next joint: id {target:2d} = {joint}  ({busname}, expected {MODEL_NAME[model]}, firmware {FW_PREFIX[model]}xx)")
        if not fw_s.startswith(FW_PREFIX[model]):
            print(f"  !! firmware family {fw_s[:2]}xx does not match {MODEL_NAME[model]} ({FW_PREFIX[model]}xx). Is this the right motor for {joint}?")
            if not a.force:
                sys.exit("  refusing; pass --force if you are sure")
        if not a.yes and input("  type 'yes' to assign and save to flash: ").strip() != "yes":
            sys.exit("aborted")

        if cur != target:
            if not b.write_reg(cur, 8, target):
                sys.exit("  write ESC_ID: no ack")
            time.sleep(0.2)
            if b.read_reg(target, 8) != target:
                sys.exit("  motor does not answer on the new id; nothing saved — power-cycle and retry")
        if not _set_master_and_save(b, target, target + MASTER_OFFSET):
            sys.exit("  FAILED setting master id / saving")
        st = b.status(target, limits(b, target))
        print(f"  feedback now on 0x{st['rx_id']:03X}" if st else "  (no status frame after save)")

        entries = [e for e in entries if e["id"] != target]
        entries.append(dict(id=target, joint=joint, bus=busname, model=MODEL_NAME[model], fw=fw_s,
                            sn=fmt_u(sn) if sn is not None else None, adapter=b.dev.serial_number,
                            date=datetime.datetime.now().isoformat(timespec="seconds")))
        save_log(sorted(entries, key=lambda e: e["id"]))
        nxt = next((i for i in JOINTS if i not in {e["id"] for e in entries}), None)
        print(f"  logged. Now: power-cycle the motor, run `check --id {target}`, unplug it."
              + (f"  Next up: id {nxt} = {JOINTS[nxt][0]}." if nxt else "  All joints done."))
    finally:
        b.close()



def cmd_move(a):
    b = Bus(pick_adapter(a.adapter))
    try:
        mid = a.id
        lim = limits(b, mid)
        st = b.status(mid, lim)
        if st is None:
            sys.exit("no status from motor")
        if st["err"] >= 8:
            sys.exit(f"motor is in error state {st['err']} ({ERR.get(st['err'], '?')}); clear it first")
        start = st["pos"]
        print(f"motor {mid} ({JOINTS.get(mid, ('?',))[0]}): start pos {start:+.4f} rad -> step {a.delta:+.2f} rad, kp {a.kp} kd {a.kd}, {a.seconds:.1f} s each way")
        b.enable(mid)
        time.sleep(0.1)
        worst = 0.0
        try:
            for target, label in ((start + a.delta, "out"), (start, "back")):
                end = time.time() + a.seconds
                last = None
                while time.time() < end:
                    b.mit(mid, lim, p=target, v=0.0, kp=a.kp, kd=a.kd, t=0.0)
                    st = b.status(mid, lim, refresh=False, timeout=0.05)
                    if st:
                        last = st
                        if st["err"] >= 8:
                            raise RuntimeError(f"fault during move: {ERR.get(st['err'], st['err'])}")
                        worst = max(worst, abs(st["tau"]))
                    time.sleep(0.005)
                err = (last["pos"] - target) if last else float("nan")
                print(f"  {label:4s}: target {target:+.4f}  reached {last['pos'] if last else float('nan'):+.4f}  "
                      f"error {err:+.4f} rad  tau {last['tau'] if last else 0:+.2f} Nm  MOS {last['mos'] if last else '?'} C")
        finally:
            b.disable(mid)
            time.sleep(0.1)
            b.drain()
        off = b.status(mid, lim)
        print(f"  disabled -> state {off['err'] if off else '?'} ({state_text(mid, off['err']) if off else '?'}), peak |tau| {worst:.2f} Nm")
    finally:
        b.close()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--adapter", help="USB serial suffix of the roboto_usb2can channel (auto if exactly one bus is live)")
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("scan"); s.add_argument("--max-id", type=int, default=32)
    for name in ("info", "check", "fix-master", "test", "zero"):
        s = sub.add_parser(name); s.add_argument("--id", type=int, required=True)
        if name == "test":
            s.add_argument("--seconds", type=float, default=3.0)
    s = sub.add_parser("set-id"); s.add_argument("--id", type=int, required=True); s.add_argument("--new", type=int, required=True)
    s.add_argument("--yes", action="store_true")
    s = sub.add_parser("next"); s.add_argument("--id", type=int, help="override: assign this id instead of the next unassigned one")
    s.add_argument("--yes", action="store_true"); s.add_argument("--force", action="store_true")
    sub.add_parser("log")
    s = sub.add_parser("move"); s.add_argument("--id", type=int, required=True)
    s.add_argument("--delta", type=float, default=0.3); s.add_argument("--kp", type=float, default=10.0)
    s.add_argument("--kd", type=float, default=0.5); s.add_argument("--seconds", type=float, default=1.5)
    a = p.parse_args()
    {"scan": cmd_scan, "info": cmd_info, "check": cmd_check, "fix-master": cmd_fix_master,
     "set-id": cmd_set_id, "test": cmd_test, "zero": cmd_zero, "next": cmd_next, "log": cmd_log, "move": cmd_move}[a.cmd](a)


if __name__ == "__main__":
    main()
