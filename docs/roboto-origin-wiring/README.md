# Roboto Origin wiring guide

Open `index.html` in a browser (no build step, no external dependencies).

- Source: https://roboparty.feishu.cn/wiki/Nop3wgiG1iRHjfkgjnLc3dUjn6F (萝博头原型机（RoboParty Roboto Origin）走线说明), extracted 2026-10-12. Feishu shows it last modified 2 Sep 2026.
- Content: English translation of the harness BOM, motor-to-motor CAN/power chain per limb (Slave IDs), waist link, battery / charging port / main switch / E-stop / three-in-one power board / RDK X5 installation, board-to-board cabling, and harness/port fixing. Original Chinese is kept in collapsible blocks.
- For agents: start with the "Quick reference" table and the "Joint map" table (23 joints: name, Slave ID, Master ID, CAN bus) at the top of `index.html`, and the "Translator's notes" section near the bottom (BOM gaps, CAN channel ↔ limb mapping).

## Images

`images/` holds the 40 original photos/diagrams from the source document, numbered in document order (original bytes, no re-encoding). Captions in `index.html` are derived from the surrounding text.

| # | file | shows (from surrounding text) | source size |
|---|---|---|---|
| 01 | `01-wiring-overview.jpg` | joint overview: Slave/Master/component ID for all 23 motors | 1230×1280 |
| 02 | `02-leg01-0x07-0x08.png` | Leg01 harness 0x07→0x08 | 343×558 |
| 03 | `03-leg02-0x08-0x09.png` | Leg02 harness 0x08→0x09 | 554×415 |
| 04 | `04-leg03-0x09-0x0a.png` | Leg03 harness 0x09→0x0A | 553×738 |
| 05 | `05-leg04-0x0a-0x0b.png` | Leg04 harness 0x0A→0x0B | 529×706 |
| 06 | `06-leg05-0x0b-0x0c.png` | Leg05 harness 0x0B→0x0C | 321×563 |
| 07 | `07-waist-overview.png` | pelvis render: waist motor and IMU location | 558×314 |
| 08 | `08-body01-0x0d-0x07.png` | Body01 harness 0x0D→0x07 | 553×738 |
| 09 | `09-arm01-0x0e-0x0f.png` | Arm01 harness 0x0E→0x0F | 553×738 |
| 10 | `10-arm02-0x0f-0x10.png` | Arm02 harness 0x0F→0x10 | 553×738 |
| 11 | `11-arm03-0x10-0x11.png` | Arm03 harness 0x10→0x11 | 366×697 |
| 12 | `12-arm04-0x11-0x12.png` | Arm04 harness 0x11→0x12 | 553×738 |
| 13 | `13-battery-foam-padding.jpg` | foam pads on battery | 960×1280 |
| 14 | `14-battery-in-place.jpg` | battery seated on base | 960×1280 |
| 15 | `15-charge-port-back-panel.jpg` | charging port on back panel (grid, left) | 959×1280 |
| 16 | `16-charge-port-detail.png` | charging port detail (grid, right) | 157×145 |
| 17 | `17-switch-estop-soldering.png` | soldering main switch connector + E-stop | 553×738 |
| 18 | `18-switch-wire-order-a.png` | backplate silkscreen: switch, E-stop, USB2CAN headers (grid, left) | 1280×810 |
| 19 | `19-switch-wire-order-b.png` | wire order (grid, right) | 491×584 |
| 20 | `20-switch-spec.png` | switch pinout by wire colour (grid, left) | 545×213 |
| 21 | `21-switch-photo.png` | switch, real part (grid, right) | 414×433 |
| 22 | `22-pcb-backplate-silkscreen.jpg` | PCB backplate silkscreen (grid, left) | 727×618 |
| 23 | `23-pcb-backplate-connection.png` | PCB backplate real connection (grid, right) | 592×552 |
| 24 | `24-carrier-m3x16-bottom.png` | carrier fixed with M3×16 from below | 456×517 |
| 25 | `25-power-board-mount-a.png` | three-in-one board fixing (grid, left) | 415×554 |
| 26 | `26-power-board-mount-b.png` | three-in-one board fixing (grid, right) | 738×553 |
| 27 | `27-standoffs-a.png` | standoffs / raising (grid, left) | 401×409 |
| 28 | `28-standoffs-b.png` | standoffs / raising (grid, right) | 378×402 |
| 29 | `29-rdkx5-mount-a.png` | RDK X5 on carrier (grid, left) | 738×553 |
| 30 | `30-rdkx5-mount-b.png` | RDK X5 on carrier (grid, right) | 554×415 |
| 31 | `31-power-board-can-connections.png` | CAN0–CAN3 on power board | 694×538 |
| 32 | `32-usb-to-typec-a.png` | USB→Type-C to USB-to-CAN (grid, left) | 648×402 |
| 33 | `33-usb-to-typec-b.png` | USB→Type-C to USB-to-CAN (grid, right) | 577×517 |
| 34 | `34-typec-to-typec-a.png` | Type-C→Type-C 5 V (grid, left) | 502×519 |
| 35 | `35-typec-to-typec-b.png` | Type-C→Type-C 5 V (grid, right) | 701×638 |
| 36 | `36-imu-gamepad-receiver-usb.png` | IMU + gamepad receiver on RDK X5 USB | 752×493 |
| 37 | `37-power-board-layout-a.png` | power board port layout (grid, left) | 704×435 |
| 38 | `38-power-board-layout-b.png` | power board port layout (grid, right) | 719×469 |
| 39 | `39-harness-clip.png` | harness clip at port | 562×473 |
| 40 | `40-can-port-uv-glue.png` | CAN port with UV glue | 553×363 |
