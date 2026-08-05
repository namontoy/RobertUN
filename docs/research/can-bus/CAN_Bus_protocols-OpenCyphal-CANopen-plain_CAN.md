# CAN Bus protocols for a Mars rover: OpenCyphal vs CANopen vs plain CAN

**For an 8-node educational rover with STM32 motor controllers and a Jetson Orin Nano, CANopen with CANopenNode is the strongest choice.** It delivers battle-tested motor control profiles (CiA 402), a turnkey ROS 2 integration via ros2_canopen, proven STM32 HAL support, and the largest open-source ecosystem — all of which reduce development time for an experienced embedded developer. OpenCyphal (formerly UAVCAN v1) offers a more elegant, modern architecture and a smaller footprint, but its steeper learning curve, lack of ROS 2 integration, and thinner community make it harder to justify for a project with a limited time budget. Plain CAN remains a viable option for the simplest version of this rover, but you start reinventing protocol features around the 8-node mark. Below is a deep analysis of each option to support that recommendation.

---

## OpenCyphal: the modern, aerospace-born pub/sub protocol

### Origins and version history

OpenCyphal began life as UAVCAN ("Uncomplicated Application-level Vehicular Computing And Networking"), created by **Pavel Kirienko** of Zubax Robotics in 2014. The protocol went through a significant evolution:

| Version | Year | Key characteristics |
|---------|------|-------------------|
| UAVCAN v0 | 2014–2020 | CAN-only; data types hardwired to fixed topic IDs; adopted by PX4 and ArduPilot |
| UAVCAN v1.0 | 2020 | Major redesign: transport-agnostic (CAN, UDP, serial), runtime-configurable port IDs, formal specification |
| Cyphal / OpenCyphal | March 2022 | UAVCAN v1 rebranded; UAVCAN v0 forked separately as "DroneCAN" |

The v0→v1 split is the single most important fact to understand. **DroneCAN** (the v0 fork) is what PX4 and ArduPilot actually use today — it is simpler, CAN-only, and has a large drone hardware ecosystem. **Cyphal** (v1) is the forward-looking protocol with a rigorous formal specification, multi-transport support, and a data-centric publish/subscribe (DCPS) architecture. The two are **not wire-compatible**. The current specification revision is **v1.0 (revised 2025-05-16)**, freely available at [opencyphal.org/specification](https://opencyphal.org/specification/).

### Core architecture

Cyphal uses **7-bit node IDs** (0–127, supporting up to 128 nodes), a **publish/subscribe model** with 13-bit Subject IDs (0–8191) for messages, and **request/response services** with 9-bit Service IDs (0–511). The only mandatory function is broadcasting a heartbeat message.

The protocol's defining feature is **DSDL** (Data Structure Description Language), a domain-specific language for defining message types with bounded arrays, unions, version management, and SI-unit annotations. The Nunavut transpiler converts DSDL definitions into C/C++/Python structs with serialization functions. This provides compile-time type safety across heterogeneous nodes — a meaningful advantage over hand-rolled serialization.

On CAN, Cyphal uses **29-bit extended frame IDs** encoding priority (3 bits), service/message flag, subject or service ID, and source node ID. A **tail byte** in each frame's payload handles multi-frame transfer reassembly, leaving **7 usable bytes per Classic CAN frame** (vs. 8 on plain CAN). Multi-frame transfers automatically fragment larger payloads with CRC-16 protection.

### Implementation on STM32

The primary library is **libcanard** (~399 GitHub stars, MIT license, ~1,000 lines of C11). The canard.h header states a minimum requirement of **32K ROM and 8K RAM**. In practice, a minimal Cyphal node (heartbeat + a few subscriptions) consumes approximately:

- **libcanard core**: ~4–6 KB Flash, ~1 KB static RAM
- **Nunavut-generated serializers**: ~2–10 KB Flash (depends on data types)
- **STM32 bxCAN driver**: ~2–3 KB Flash
- **Total**: roughly **10–20 KB Flash, 2–4 KB RAM** plus RX/TX buffers

This fits comfortably on an **STM32F103** (64 KB Flash / 20 KB RAM), though it's tighter than CANopenNode's minimal configuration. The library works **completely bare-metal** — no OS, no threads, no interrupts required. The bxCAN driver in [platform_specific_components](https://github.com/OpenCyphal-Garage/platform_specific_components) accesses registers directly (not via STM32 HAL), which is powerful but means no CubeMX integration. A basic working node requires approximately **100–200 lines of application code** plus the Nunavut build step.

**Critical caveat**: the official bxCAN driver supports STM32F1/F2/F3/F4 families. There is **no official FDCAN driver** for newer STM32G4/H7 — you would need community code or write your own.

### Ecosystem and tooling

| Tool | Purpose | Stars |
|------|---------|-------|
| [libcanard](https://github.com/OpenCyphal/libcanard) | C transport library (bare-metal) | ~399 |
| [pycyphal](https://github.com/OpenCyphal/pycyphal) | Python stack (Linux/Jetson) | ~134 |
| [nunavut](https://github.com/OpenCyphal/nunavut) | DSDL → C/C++/Python code generator | ~46 |
| [yakut](https://github.com/OpenCyphal/yakut) | CLI diagnostics, configuration, firmware update | ~61 |
| [libcyphal](https://github.com/OpenCyphal-Garage/libcyphal) | C++ higher-level library (beta) | ~272 |

**PyCyphal** on the Jetson Orin Nano works via SocketCAN (with a USB-CAN adapter like PEAK PCAN-USB or Zubax Babel) and provides a full async Python API for subscribing to all bus traffic, sending commands, and managing node configuration. It's well-documented at [pycyphal.readthedocs.io](https://pycyphal.readthedocs.io).

### Real-world usage and community

Cyphal's primary deployment vector is the **drone ecosystem** — PX4 and ArduPilot support it alongside DroneCAN, though DroneCAN remains the default recommendation. Wikipedia cites deployments in "spacecraft, underwater robots, racing cars, and micromobility vehicles," but specific spacecraft names are not public. The community is **small but technically excellent**: the [OpenCyphal forum](https://forum.opencyphal.org/) is active with Pavel Kirienko himself responding to questions, but post volume is modest. **No known rover project documents using Cyphal.** No official ROS 2 package exists — integration requires custom Python bridging via PyCyphal.

---

## CANopen: the industrial workhorse with 30 years of heritage

### Standards and governance

CANopen is maintained by **CAN in Automation (CiA)**, an international non-profit founded in 1992 in Nuremberg. The core specification is **CiA 301 v4.2.0** (2011), internationally standardized as **EN 50325-4**. The motion control profile **CiA 402** (also known as DS402) defines a standardized state machine, control/status words, and operating modes for servo drives and stepper motors — and is partly standardized in IEC 61800-7.

CiA 301 can be downloaded free after registration on [can-cia.org](https://www.can-cia.org/cia-groups/technical-documents), though CiA 402 requires membership. In practice, both protocols are thoroughly documented by third-party tutorials, open-source code, and vendor manuals — the paywall is rarely a showstopper.

### Core architecture in detail

CANopen's centerpiece is the **Object Dictionary (OD)**, a structured array of variables indexed by 16-bit index and 8-bit sub-index. Every parameter — communication settings, PDO mappings, application data — lives in the OD. This is simultaneously CANopen's greatest strength (everything is introspectable and configurable) and its steepest learning curve.

The protocol uses **11-bit standard CAN IDs** divided into a 4-bit function code and 7-bit node ID (1–127). The pre-defined connection set assigns COB-IDs predictably:

- **PDOs (Process Data Objects)**: Zero-overhead real-time data. Each node gets 4 TPDOs and 4 RPDOs by default. PDO payloads carry raw mapped OD variables — **no protocol overhead**, full 8 bytes available. PDOs can be synchronized to the SYNC message (e.g., 100 Hz) or event-driven.
- **SDOs (Service Data Objects)**: Client/server read/write access to any OD entry. Used for configuration, parameter tuning, and diagnostics. Supports expedited (≤4 bytes), segmented, and block transfer modes.
- **NMT (Network Management)**: Master/slave state machine with Pre-operational → Operational → Stopped states. Heartbeat protocol for node monitoring. Boot-up messages for discovery.
- **EMCY (Emergency)**: Standardized fault reporting — each node broadcasts error codes with a manufacturer-specific payload.
- **SYNC**: Broadcast timing reference for coordinated PDO exchange across all nodes.

For motor control, **CiA 402** defines a state machine (Not Ready → Switch On Disabled → Ready to Switch On → Switched On → Operation Enabled) controlled via **Controlword** (OD index 0x6040) and monitored via **Statusword** (0x6041). Operating modes include **Cyclic Synchronous Position (CSP)**, **Cyclic Synchronous Velocity (CSV)**, Profile Position, Profile Velocity, and Homing — precisely the modes a 6-wheeled rover needs.

### Implementation on STM32

**CANopenNode** ([github.com/CANopenNode/CANopenNode](https://github.com/CANopenNode/CANopenNode), ~1,800 stars, Apache 2.0) is the primary open-source implementation. Its companion project **CanOpenSTM32** ([github.com/CANopenNode/CanOpenSTM32](https://github.com/CANopenNode/CanOpenSTM32), ~405 stars) provides **direct STM32 HAL integration** with auto-detection of bxCAN vs. FDCAN peripherals. Working examples exist for STM32F4-Discovery, STM32G0C1-EV, STM32F076 Nucleo, and STM32H735G-DK. A community BluePill (STM32F103) example exists at [github.com/viteo/STM32-BluePill-CANOpenNode](https://github.com/viteo/STM32-BluePill-CANOpenNode).

Memory footprint is larger than libcanard due to the full protocol stack: approximately **15–30 KB Flash and 4–8 KB RAM** for a basic node with a few PDOs and an SDO server. This is tight but feasible on an STM32F103 (64 KB / 20 KB) and very comfortable on F4/G4/H7 parts.

Setup follows a well-documented path: generate a CubeMX project with CAN enabled, add CANopenNode as a submodule, generate OD files using the open-source CANopenEditor, and call `canopen_app_init()` / `canopen_app_process()` from your main loop. A **3-hour video tutorial** walks through the entire process. Estimated time to first heartbeat: **a few hours** with examples.

### ROS 2 integration: CANopen's decisive advantage

**ros2_canopen** ([github.com/ros-industrial/ros2_canopen](https://github.com/ros-industrial/ros2_canopen)), maintained by Fraunhofer IPA under the ROS-Industrial Consortium, provides a turnkey bridge between ROS 2 and CANopen devices. It supports ROS 2 Humble through Rolling, is built on the professional-grade **lely-core** CANopen library, and offers:

- YAML-based bus configuration with EDS file references per device
- Full **CiA 402 driver** with ros2_control integration (position, velocity, torque modes)
- Lifecycle-managed nodes (configure → activate → deactivate)
- Plugin-based custom driver architecture

On the Jetson Orin Nano, setup involves configuring SocketCAN (via USB-CAN adapter), installing ros2_canopen, defining your bus topology in YAML, and launching the DeviceContainer. **This is the only CAN protocol with a maintained, feature-rich ROS 2 package.** OpenCyphal has no equivalent — ROS 2 integration requires writing custom PyCyphal bridge nodes.

### Real-world robotics deployments

CANopen's deployment footprint dwarfs OpenCyphal's. **Universal Robots** cobots use CAN internally. KUKA, FANUC, and ABB industrial robots interface with CANopen devices. CiA 402 motor drives from Maxon, Faulhaber, Nanotec, and dozens of others are standard in robotics. **Lely Industries** runs thousands of agricultural robots daily on the lely-core CANopen stack. The **CORC** (CANopen Robot Controller) framework from University of Melbourne uses CANopen for rehabilitation exoskeletons.

Most critically for this rover: **ESA's ExoMars Rosalind Franklin rover uses CAN with a CANopen-based protocol** (ECSS-E-ST-50-15C), making it the **first space mission to fly CANopen-derived protocol in space**. ESA also funded pre-qualification of lely-core to ECSS criticality B for satellite use, documented at [canopen.space](https://canopen.space/).

---

## Head-to-head: which protocol fits this rover

### Philosophy and design trade-offs

OpenCyphal was designed as a **peer-to-peer, masterless** protocol for safety-critical aerospace systems. It prioritizes type safety, transport agnosticism, and formal correctness. CANopen was designed as a **master/slave industrial fieldbus** with comprehensive device profiles. It prioritizes interoperability, configuration management, and standardized behavior across vendors.

For a rover with a clear central computer (Jetson) commanding peripheral nodes (STM32 motor controllers), CANopen's master/slave model maps naturally to the architecture. Cyphal's masterless pub/sub is more elegant but doesn't provide proportional benefit when you already have a designated master.

### Implementation complexity on constrained MCUs

| Factor | OpenCyphal (libcanard) | CANopen (CANopenNode) |
|--------|----------------------|----------------------|
| Flash footprint | ~10–20 KB | ~15–30 KB |
| RAM footprint | ~2–4 KB | ~4–8 KB |
| STM32 HAL integration | Register-level (no HAL) | Direct HAL support |
| CubeMX compatibility | No | Yes |
| FDCAN support | No official driver | Auto-detected |
| Board examples | Community only | 4+ official boards |
| Build toolchain | Requires Nunavut (Python) | Standard C build |
| Fits STM32F103 | Yes (comfortable) | Yes (tight but proven) |
| Time to first node | Days (steeper setup) | Hours (with tutorial) |

### ROS 2 and Jetson integration

CANopen wins decisively. ros2_canopen provides lifecycle nodes, CiA 402 motor control, ros2_control hardware interfaces, and YAML-based configuration. OpenCyphal requires manual PyCyphal-to-ROS2 bridging — functional but unsupported. If the Jetson Orin Nano runs ROS 2 (likely for a research rover), this gap matters enormously.

### Community health metrics

| Metric | OpenCyphal | CANopen (CANopenNode) |
|--------|-----------|----------------------|
| GitHub stars (main lib) | ~399 | ~1,800 |
| GitHub stars (STM32 port) | ~76 | ~405 |
| Contributors | Small core team | Broader community |
| StackOverflow presence | Dozens of questions | Hundreds of questions |
| Commercial device ecosystem | Growing (drone hardware) | Massive (industrial drives) |
| Tutorial quality | Specification + forum | Videos, CSS Electronics, vendor docs |

### Learning curve

Both protocols demand significant upfront learning. CANopen's Object Dictionary concept is initially confusing — understanding PDO mapping, COB-ID assignment, and the NMT state machine requires studying CiA 301. But resources are abundant: the [CSS Electronics CANopen tutorial](https://www.csselectronics.com/pages/canopen-tutorial-simple-intro) is an excellent visual introduction, and CanOpenSTM32's video tutorial walks through the entire STM32 setup.

OpenCyphal's learning curve centers on DSDL, the Nunavut build integration, Subject/Service ID configuration, and the register-based configuration model. The formal specification is rigorous but dense. The [Cyphal Guide](https://opencyphal.org/guide) helps, but there's no equivalent of CANopenNode's step-by-step video walkthrough.

---

## When plain CAN is enough — and when it isn't

Plain CAN gives you hardware arbitration, **>99.99%** error detection, automatic retransmission, and fault confinement at the silicon level. You lose: standardized message definitions, node management, multi-frame transfers, time synchronization, firmware updates over bus, device profiles, and interoperability with commercial hardware.

### Bus load reality check at 125 kbps

For this 8-node rover at 125 kbps with 6 motor controllers at 50 Hz and sensor data at 20 Hz, the estimated bus load is approximately **60–70%** — at the upper edge of the recommended 70% ceiling. This is workable but leaves little headroom for diagnostics or burst traffic. **Increasing to 250 kbps** drops load to ~35% with no hardware changes (just configuration). Any higher-level protocol adds overhead (Cyphal's tail byte costs 1 byte per frame; CANopen's PDOs have zero overhead but SDO/NMT traffic adds load).

### The breakpoint for protocol adoption

For **≤4 nodes** with simple, well-defined data, plain CAN with a hand-crafted ID table is the fastest path to working code. At **5–10 nodes**, discipline becomes important — you need documented message tables, heartbeat monitoring, and consistent serialization. At **10+ nodes**, a formal protocol pays for itself in debugging time alone.

This rover sits at the **inflection point**: 8 nodes with motor control, sensor fusion, and fault reporting. Plain CAN works if the developer is willing to implement heartbeats, watchdogs, serialization conventions, and message documentation from scratch. A protocol saves that effort and adds introspection tooling.

### What real rover projects actually use

Notably, most educational/hobby rovers **don't use CAN at all**:

- **JPL Open Source Rover**: Raspberry Pi + RoboClaw motor controllers via serial UART + ROS 2
- **Sawppy Rover**: Raspberry Pi + LX-16A serial bus servos (proprietary UART protocol)
- **ExoMy (ESA educational)**: Raspberry Pi + PCA9685 PWM servo driver over I2C + ROS

These projects optimize for accessibility and low cost, not distributed reliability. CAN makes sense when nodes are physically distributed (long cable runs), when bidirectional telemetry from each actuator matters, or when fault isolation between nodes is important — all true for a more serious rover build.

The **ESA ExoMars Rosalind Franklin** rover uses CAN with CANopen (ECSS-E-ST-50-15C) — the only rover in space with a CANopen-derived protocol. In the drone world, **PX4 recommends DroneCAN** (UAVCAN v0) for its robust wiring, bidirectional ESC telemetry, firmware updates, and node discovery. Several custom robot platforms (Commonplace Robotics, Nexi robot) use plain CAN with proprietary protocols, typically citing simplicity and fit-for-purpose design.

---

## Every URL you need

### OpenCyphal / Cyphal

| Resource | URL |
|----------|-----|
| Website | https://opencyphal.org/ |
| Specification (PDF) | https://opencyphal.org/specification/ |
| Cyphal Guide (intro textbook) | https://opencyphal.org/guide |
| GitHub organization | https://github.com/OpenCyphal |
| libcanard (C, bare-metal) | https://github.com/OpenCyphal/libcanard |
| PyCyphal (Python, Linux/Jetson) | https://github.com/OpenCyphal/pycyphal |
| PyCyphal docs | https://pycyphal.readthedocs.io |
| Nunavut (DSDL code generator) | https://github.com/OpenCyphal/nunavut |
| Yakut (CLI tool) | https://github.com/OpenCyphal/yakut |
| STM32 bxCAN driver | https://github.com/OpenCyphal-Garage/platform_specific_components |
| Standard data types (DSDL) | https://github.com/OpenCyphal/public_regulated_data_types |
| Forum | https://forum.opencyphal.org/ |
| DroneCAN (UAVCAN v0 fork) | https://dronecan.github.io/ |

### CANopen

| Resource | URL |
|----------|-----|
| CAN in Automation (CiA) | https://www.can-cia.org/ |
| CiA technical documents | https://www.can-cia.org/cia-groups/technical-documents |
| CiA 402 info | https://www.can-cia.org/can-knowledge/cia-402-series-canopen-device-profile-for-drives-and-motion-control |
| CANopenNode | https://github.com/CANopenNode/CANopenNode |
| CanOpenSTM32 | https://github.com/CANopenNode/CanOpenSTM32 |
| CANopenNode docs | https://canopennode.github.io |
| BluePill example | https://github.com/viteo/STM32-BluePill-CANOpenNode |
| lely-core (library behind ros2_canopen) | https://opensource.lely.com/canopen/ |
| ESA CANopen for space | https://canopen.space/ |
| CANopenEditor (OD GUI) | https://github.com/robincornelius/libedssharp |

### ROS 2 integration

| Resource | URL |
|----------|-----|
| ros2_canopen | https://github.com/ros-industrial/ros2_canopen |
| ros2_canopen manual | https://ros-industrial.github.io/ros2_canopen/manual/rolling/ |

### PX4 / Drones

| Resource | URL |
|----------|-----|
| PX4 CAN overview | https://docs.px4.io/main/en/can/ |
| PX4 DroneCAN docs | https://docs.px4.io/main/en/dronecan/ |

### Tutorials and references

| Resource | URL |
|----------|-----|
| CSS Electronics CANopen tutorial | https://www.csselectronics.com/pages/canopen-tutorial-simple-intro |
| CSS Electronics CAN bus intro | https://www.csselectronics.com/pages/can-bus-simple-intro-tutorial |
| CSS Electronics ultimate CAN guide (PDF) | https://www.csselectronics.com/pages/can-bus-ultimate-guide |
| Kvaser UAVCAN overview | https://kvaser.com/uavcan/ |
| Jetson CAN documentation | https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html |
| Wikipedia — Cyphal | https://en.wikipedia.org/wiki/Cyphal |
| Wikipedia — CANopen | https://en.wikipedia.org/wiki/CANopen |

---

## Conclusion: the practical recommendation

For this specific rover — 8 nodes, STM32 motor controllers, Jetson Orin Nano, an experienced embedded developer with limited time — **CANopen with CANopenNode is the recommended choice**. Three factors drive this:

**First, CiA 402 motor control is purpose-built for this problem.** The standardized state machine, controlword/statusword mechanism, and cyclic synchronous velocity mode handle exactly the wheel control loop this rover needs. You get a proven, industry-standard motor interface instead of designing one from scratch.

**Second, the STM32-to-Jetson pipeline is solved.** CANopenNode with CanOpenSTM32 gives you HAL-integrated, CubeMX-compatible firmware on the STM32 side. ros2_canopen on the Jetson gives you lifecycle-managed ROS 2 nodes with CiA 402 drivers and ros2_control integration. This end-to-end stack has no equivalent in the OpenCyphal ecosystem.

**Third, the community and documentation depth reduces risk.** With ~1,800 GitHub stars, a 3-hour video tutorial, working STM32 examples (including Blue Pill), CSS Electronics visual guides, and decades of industrial deployment, you're unlikely to hit a problem that no one has solved before.

OpenCyphal is the better protocol for a greenfield aerospace project where you control both ends, need multi-transport support (CAN + UDP + serial), and value formal type safety above ecosystem maturity. If you're planning to integrate with PX4/ArduPilot drones in the future, **DroneCAN** (the UAVCAN v0 fork, not Cyphal) is the pragmatic choice in that ecosystem.

Plain CAN is a reasonable fallback if you want to ship the fastest possible prototype. Define a simple message table, implement heartbeats manually, and get driving. But at 8 nodes with motor control, sensor fusion, and fault reporting, you'll end up reimplementing half of CANopen anyway — and without the tooling to debug it.

**Recommended rover architecture with CANopen:**

| Node | ID | Stack | Role |
|------|----|-------|------|
| Jetson Orin Nano | 1 | ros2_canopen (lely-core) via USB-CAN adapter | NMT master, command dispatch, telemetry logging |
| Wheel motors 1–6 | 2–7 | CANopenNode + CanOpenSTM32, bare-metal | CiA 402 slaves (CSV mode), encoder feedback via TPDO |
| Sensor tower | 8 | CANopenNode + CanOpenSTM32, bare-metal | Sensor readings via TPDO, configuration via SDO |

Set the bus to **250 kbps** (not 125 kbps) for comfortable headroom, use SYNC at 50–100 Hz for coordinated wheel updates, and leverage EMCY messages for per-node fault reporting. Total firmware overhead per STM32 node: ~20–30 KB Flash, ~5–8 KB RAM — well within any modern STM32 with a CAN peripheral.