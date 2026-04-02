# CAN Bus on the Jetson Orin Nano: complete hardware and software guide

**The Jetson Orin Nano has a built-in hardware CAN controller — no external SPI-to-CAN bridge or USB adapter is required.** The T234 SoC includes an NVIDIA MTTCAN (Message Time-Triggered CAN) peripheral on its Always-On block, supporting both CAN 2.0 and CAN FD. However, the developer kit does not ship with a ready-to-use CAN port; the CAN signals are routed to an **unpopulated 4-pin header (J17)** that requires soldering, and an **external 3.3V CAN transceiver** (such as the TI SN65HVD230) is always needed to connect to a physical CAN bus. With JetPack 6.x / L4T R36.5.0, enabling CAN involves loading the blacklisted `mttcan` kernel module, configuring pinmux registers, and using standard Linux SocketCAN commands — a process that takes roughly five minutes once you understand the steps.

---

## The Orin Nano SoC has native CAN hardware, but only one channel

The NVIDIA T234 SoC powering the Jetson Orin Nano contains **two MTTCAN controllers** at base addresses `mttcan@c310000` and `mttcan@c320000`. However, the Orin Nano module only routes **one CAN interface (CAN0)** through its 260-pin SO-DIMM edge connector — pins for `can1_din` and `can1_dout` are listed as "n/a" in the official pinmux documentation. This stands in contrast to the Jetson AGX Orin, which exposes both CAN0 and CAN1.

The MTTCAN controller supports **CAN 2.0** (standard and extended frames) at bitrates from **10 kbps to 1 Mbps**, and **CAN FD** with data bitrates up to **5 Mbps** using standard transceivers (theoretically 15 Mbps with TDCR tuning). The controller resides in the SoC's Always-On (AON) domain, associated with the Cortex-R5 Sensor Processing Engine. It uses NVIDIA's proprietary `mttcan` kernel driver — not the mainline Bosch M_CAN driver — integrated into the Linux SocketCAN framework so the interface appears as a standard network device.

| Specification | Value |
|---|---|
| CAN controllers exposed | **1 (CAN0 only)** |
| Protocol support | CAN 2.0A/B, CAN FD |
| Standard bitrate range | 10 kbps – 1 Mbps |
| CAN FD data bitrate | Up to 5 Mbps (15 Mbps theoretical) |
| Default CAN core clock | 50 MHz |
| Kernel driver | `mttcan` (SocketCAN) |
| Device tree node | `mttcan@c310000` |

If you need a second CAN channel, the standard workaround is adding an **MCP2515 SPI-to-CAN module** connected via the 40-pin header's SPI pins, though this requires separate device tree configuration and a different kernel driver.

---

## Pin locations: J17 header on the dev kit, SO-DIMM pins 143/145 on the module

**CAN signals are NOT available on the 40-pin expansion header (J12).** This is a common source of confusion. On the Orin Nano developer kit carrier board (P3768), CAN_TX and CAN_RX are routed exclusively to the **J17 header** — a 1×4, 2.54 mm pitch, right-angle connector footprint that **ships unpopulated**. You must solder a pin header to J17 yourself.

| J17 Pin | Signal | SO-DIMM Pin | Direction | Voltage |
|---|---|---|---|---|
| 1 | **CAN_TX** | 145 | Output (from SoC) | 3.3V |
| 2 | **CAN_RX** | 143 | Input (to SoC) | 3.3V |
| 3 | GND | — | Ground | — |
| 4 | 3.3V | — | Power supply | 3.3V |

The SoC's native I/O voltage for CAN pins is **1.8V internally**, but the Orin Nano module includes on-board level shifting so that the signals at the SO-DIMM connector (and therefore J17) operate at **3.3V**. Custom carrier board designers should plan for 3.3V logic on SO-DIMM pins 143 and 145.

For the pinmux, the Orin Nano defaults these pins to **SFIO (Special Function I/O) mode for CAN**, unlike the AGX Orin where they default to GPIO. In practice, however, most users still need to explicitly write the pinmux registers (see software setup below). The NVIDIA Jetson-IO tool (`jetson-io.py`) **does not support CAN configuration** on the Orin Nano, because Jetson-IO only manages the 40-pin header and the CAN pins live on J17. This was confirmed by NVIDIA engineer KevinFFF on the developer forums.

---

## External transceiver wiring: SN65HVD230 is the go-to choice

A CAN controller outputs logic-level TX/RX signals — it does not generate the differential CAN_H/CAN_L bus signals defined by ISO 11898-2. An external CAN transceiver is always mandatory.

NVIDIA explicitly recommends the **Waveshare SN65HVD230 CAN Board** for development. This module uses the Texas Instruments SN65HVD230 transceiver IC, which operates at 3.3V and supports classical CAN up to 1 Mbps. Wiring from J17 is straightforward:

| J17 Pin | Connects to transceiver |
|---|---|
| Pin 1 (CAN_TX) | TXD input |
| Pin 2 (CAN_RX) | RXD output |
| Pin 3 (GND) | GND |
| Pin 4 (3.3V) | VCC |

The transceiver board then provides screw terminals or pads for **CAN_H**, **CAN_L**, and **GND** to connect to your CAN bus. Ensure proper **120Ω termination** at both ends of the bus — many Waveshare SN65HVD230 boards include a solder jumper to enable a built-in 120Ω termination resistor.

For production designs or CAN FD applications, compatible alternatives include the **TJA1051T/3** (NXP, 3.3V), **MCP2562FD** (Microchip, 3.3V/5V split supply, CAN FD rated), and **TCAN1042** (TI, CAN FD). The key requirement is **3.3V logic-level compatibility**. Avoid 5V-only transceivers like the MCP2551 or TJA1050 without a level shifter.

---

## Software setup: three steps to a working CAN0 interface

Enabling CAN on JetPack 6.x (L4T R36.x, Ubuntu 22.04) requires configuring pinmux, loading the kernel module, and bringing up the SocketCAN interface. Here is the complete procedure:

**Step 1 — Install prerequisites:**
```bash
sudo apt-get install busybox can-utils
```

**Step 2 — Configure pinmux registers** (non-persistent; must repeat after each reboot):
```bash
sudo busybox devmem 0x0c303018 w 0xc458   # CAN0_DIN (RX)
sudo busybox devmem 0x0c303010 w 0xc400   # CAN0_DOUT (TX)
```

**Step 3 — Load kernel modules:**
```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
```

The `mttcan` module is **blacklisted by default** via `/etc/modprobe.d/denylist-mttcan.conf`. To enable auto-loading at boot, remove this file: `sudo rm /etc/modprobe.d/denylist-mttcan.conf` and add `mttcan` to `/etc/modules-load.d/mttcan.conf`.

**Step 4 — Bring up the interface with SocketCAN:**
```bash
# Classical CAN at 500 kbps:
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Or CAN FD with 500 kbps nominal / 1 Mbps data rate:
sudo ip link set can0 up type can bitrate 500000 dbitrate 1000000 berr-reporting on fd on
```

**Step 5 — Test with can-utils:**
```bash
# Receive frames:
candump can0

# Send a frame (ID 0x123, data 0xABCDABCD):
cansend can0 123#ABCDABCD

# Loopback self-test (no transceiver needed — short TX and RX pins):
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 loopback on
sudo ip link set can0 up
candump can0 &
cansend can0 123#ABCDABCD
```

To verify the device tree node is enabled: `cat /proc/device-tree/bus@0/mttcan@c310000/status` should return `okay`. Note the `bus@0/` prefix is specific to JetPack 6.x — on JetPack 5.x the path is `/proc/device-tree/mttcan@c310000/status`.

For **persistent pinmux configuration** in production, use the NVIDIA Pinmux Config Template spreadsheet (available from the Jetson Download Center), configure CAN0_DOUT and CAN0_DIN to their CAN function, generate the `.dtsi` output, place it in `<Linux_for_Tegra>/bootloader/generic/BCT/tegra234-mb1-bct-pinmux-p3767-dp-a03.dtsi`, and reflash the board. This applies pinmux at the MB1 boot stage, making it permanent. An alternative for development is to place the `devmem` commands in a systemd service or `/etc/rc.local`.

---

## Known limitations, bugs, and version-specific gotchas

Several practical issues consistently surface in the NVIDIA developer forums and community reports. The most important ones to know:

**The mttcan blacklist is the number-one stumbling block.** Nearly every "CAN doesn't work" forum post traces back to users not realizing the kernel module is blacklisted by default. Removing `/etc/modprobe.d/denylist-mttcan.conf` is the essential first fix.

**Pinmux resets on every reboot** when using the `busybox devmem` method. For anything beyond bench testing, flash the pinmux via the MB1 BCT dtsi method or create a systemd service that runs the devmem commands at boot.

**JetPack 6.0 Developer Preview had CAN issues** related to pinmux configuration changes, but these were resolved in JetPack 6.1 and later. On **JetPack 6.5 / L4T R36.5.0** specifically, the release notes contain no CAN-specific changes — the mttcan subsystem has been stable across the JetPack 6.x series.

**CAN FD at 5 Mbps may require TDCR tuning.** Some users report CAN FD failures at high data bitrates even with FD-rated transceivers. The Transmitter Delay Compensation Register must be configured via sysfs — note the path changed between JetPack versions: `/sys/devices/c310000.mttcan/net/can0/tdcr` on JP5.x versus `/sys/devices/platform/bus@0/c310000.mttcan/net/can0/tdc_offset` on JP6.x.

**Timing synchronization issues** have been reported on the Orin Nano Super variant when communicating with external devices, even when loopback tests pass. Adding the `sjw 4` (Synchronization Jump Width) parameter to the `ip link` command resolves this in many cases: `sudo ip link set can0 type can bitrate 500000 sjw 4`.

**"No buffer space available" errors** during sustained high-rate sending can be worked around by using polling mode: `cangen -L 8 can0 -p 1000`. A **potential CAN FD DLC limitation** of 8 bytes in the hardware receive path has been discussed (referencing section 9.4.1.2 of the Orin TRM), though this requires further verification for your specific use case.

---

## Official documentation and community resources worth bookmarking

The most authoritative and useful sources, ranked by importance:

- **NVIDIA Jetson Linux Developer Guide — Controller Area Network (CAN)** for L4T R36.x: the single most important reference, covering all Orin platforms with exact pinmux values, commands, and platform-specific details. Available at `docs.nvidia.com/jetson/archives/r36.2/DeveloperGuide/HR/ControllerAreaNetworkCan.html`.
- **Jetson Orin Nano Developer Kit Carrier Board Specification** (SP-11324-001_v1.3): documents the J17 header pinout, SO-DIMM CAN pin mapping, and carrier board schematic.
- **NVIDIA Developer Forums** (`forums.developer.nvidia.com`): the most active troubleshooting resource. Key threads include "How to use the CAN of Jetson Orin Nano?" (April 2023, 7,000+ views), "CAN-bus functionality on Jetson Orin Nano dev kit" (September 2024, confirmed working setup), and "Jetson Orin Nano FDCAN at 5M" (CAN FD high-speed configuration). NVIDIA engineer **KevinFFF** is the most responsive official responder on CAN-related threads.
- **Third-party carrier boards** offer production-ready CAN: Seeed Studio's J401 and A608, Auvidea's JNX42/JNX45/JNX110 (popular for drone applications), and Forecr's DSBOARD-ORNX series all include integrated CAN transceivers and proper connectors, eliminating the J17 soldering requirement.
- **Forecr CAN Bus tutorials** (`forecr.io/blogs/connectivity/dsboard-ornx-canbus-interface-tutorial`) provide step-by-step guides tested on Orin Nano carrier boards.

## Conclusion

The Jetson Orin Nano's built-in MTTCAN controller is genuinely capable hardware — CAN 2.0 and CAN FD are fully supported with a mature SocketCAN driver. The practical friction lies entirely in the developer kit's physical design: an unpopulated header, the need for an external transceiver, a blacklisted kernel module, and non-persistent pinmux configuration. For robotics or automotive prototyping, budget 30 minutes for the initial hardware setup (soldering J17, wiring the SN65HVD230) and software configuration. For production, strongly consider a third-party carrier board with an integrated CAN transceiver and connector — Auvidea and Seeed Studio both offer compact, well-documented options. The software stack on JetPack 6.5 is stable and requires no patches or workarounds beyond the standard module loading and pinmux steps documented here.