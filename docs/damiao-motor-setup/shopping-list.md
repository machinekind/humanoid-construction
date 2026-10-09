# Shopping list — DAMIAO DM-J4340P-2EC bench setup (Wrocław / Poland)

Researched 2026-10-09. Prices and stock change; verify before ordering.

## What the motor exposes (from the DM-J4340-2EC user manual)

| Port | Connector | Pins | Notes |
|---|---|---|---|
| Power + CAN (x2, daisy-chainable) | **XT30 (2+2)** socket | VCC, GND + CAN_L, CAN_H | Motor ships with one XT30(2+2)-F single-ended cable, 100 mm |
| Debug serial | **JST GH1.25, 3-pin** | GND, RX, TX (3.3 V UART) | Motor ships with one GH1.25 3-pin cable, 300 mm |

Control interface: CAN @ 1 Mbps. Parameter interface: UART @ 921600 bps.

## Which USB adapter

DM_Motor_Tool has two transport modes, and the official host-software manual (v1.0, 2025-03-28) is explicit:

- **UART mode**: "通用串口设备都可以使用，不局限于USB2CAN" — any generic USB-serial device works. Covers: read version, firmware upgrade, read/write/import/export parameters, parameter calibration, motor-side and output-shaft encoder calibration, CAN ID set.
- **CAN mode**: "涉及到 CAN 调试的功能需要使用 USB2CAN 模块" — functions that go over CAN need the DAMIAO USB2CAN module. Covers: enable/disable, live control (MIT/position/speed), **save zero point**, CAN-side reads.

So:

1. **DAMIAO DM-USB-CAN** (or the newer **DM-USB-CANFD**) — the only adapter that works with DM_Motor_Tool for the CAN-mode steps (section 3.2 "zero position" and any live test). Specs: USB-C, CH340 virtual COM port, GH1.25 2-pin CAN + GH1.25 3-pin UART headers, 39×18 mm (CANFD: 48×18 mm, 128 KB buffer).
2. **Generic USB-UART 3.3 V** (CP2102 / FT232, must do 921600 bps) — buyable in Poland today; enough for sections 1, 2, 3.1 and 4 of the guide (calibration, parameters, ID, firmware) via the GH1.25 3-pin debug port.
3. *(Later, for Linux/ROS control by agents, not for DM_Motor_Tool)*: a SocketCAN adapter such as CANable/candleLight, or reuse the DAMIAO adapter through its serial protocol (DM_Motor_Control Linux lib, /dev/ttyACM*, 921600).

## Where to buy

| Item | Source | Price | Lead time |
|---|---|---|---|
| DM-USB-CAN (DAMIAO) | OpenELAB (DE, EU stock/DDP) https://openelab.io/products/damiao-brushless-servo-usb-can | €24.95 + €7.95 ship | 3–5 business days after dispatch; confirm stock, some SKUs ship from CN |
| DM-USB-CANFD (DAMIAO) | OpenELAB https://openelab.io/products/damiao-brushless-servo-usb-to-fdcan | ~€30 | same |
| Seeed "DAMIAO 43 USB-CAN driver board" | MyBotShop.de | €39.95 | 45 days — avoid |
| Same module from CN | Seeed / Foxtech / AIFitLab / Taobao | $15–23 | 2–4 weeks, often backordered |
| Polish shops / Allegro | — | — | **not stocked anywhere in PL** (checked Allegro, Botland, Kamami) |
| USB-UART CP2102, USB-C, 3.3 V | Kamami (Waveshare 20644) https://kamami.pl/en/converters-usb-uart-rs232/587417-cp2102-usb-uart-board-type-c-usb-uart-cp2102-converter-with-usb-type-c-connector.html | ~30–40 zł | next day |
| USB-UART CP2102 (cheapest) | Gotronik https://www.gotronik.pl/konwerter-usb-ttluartrs232-cp2102-kartapdf-11266.html | 18 zł | 1–2 days |
| USB-UART FT232RL 3.3/5 V jumper | Kamami https://kamami.pl/en/converters-usb-uart-rs232/580365-usb-uart-converter-with-ft232rl-chip-modft232rl-5906623473793.html | ~40 zł | next day |
| Bench PSU 0–60 V / 5 A (48 V motor!) | Botland Korad U206 (KOR-24361) https://botland.com.pl ; Kamami Wanptek KPS605D | ~450–700 zł | next day |
| JST GH 1.25 2-pin and 3-pin pre-crimped leads | Kamami "JST cables" https://kamami.pl/en/15353-jst-cables | few zł | next day |
| XT30 / XT30(2+2) male plugs, 120 Ω resistor, USB-C data cable | Botland / Kamami / any local electronics shop | few zł | — |

No stationary shop in Wrocław was found that stocks any of the adapters. Botland's warehouse is ~55 km away (Gola Dzierżoniowska) and both Botland and Kamami deliver next day to Wrocław parcel lockers.

## Recommended order

- **Order today, EU**: 1× DM-USB-CAN (or CANFD) from OpenELAB.
- **Order today, PL (arrives tomorrow)**: 1× CP2102 USB-UART 3.3 V, 1× 0–60 V/5 A bench PSU (if none in the lab), GH1.25 2-pin + 3-pin leads, XT30 plugs, a USB-C data cable.
- With the UART adapter alone you can already do sections 1–2, 3.1 and 4 of the guide; section 3.2 (enable + save zero) waits for the DAMIAO adapter.
