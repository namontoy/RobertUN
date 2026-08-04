# CAN Bus and STM32 CAN: a curated resource guide

**For an experienced embedded developer targeting STM32F103 and STM32F4xx, the best CAN Bus learning path combines five standout resources**: the CSS Electronics visual guides for conceptual grounding, TI's SLLA270 for physical layer mastery, the ST Community knowledge base articles (by mƎALLEm) for bxCAN peripheral configuration, the bittiming.can-wiki.info calculator for register-level bit timing, and the OpenCyphal/UAVCAN bare-metal bxCAN driver as a production-quality register-level reference. Every resource below is freely available online unless explicitly noted. This guide is organized into two categories — general CAN protocol understanding and STM32-specific implementation — with resources ranked by quality and annotated for depth level.

---

## Category 1: CAN Bus protocol, theory, and physical layer

### The Bosch specification and foundational references

The **Bosch CAN Specification 2.0** (1991) remains the authoritative protocol definition. This ~72-page PDF covers every aspect of the CAN protocol: frame types (Data, Remote, Error, Overload), non-destructive bitwise arbitration, NRZ encoding with bit stuffing, CRC calculation, error detection, and fault confinement. Part A defines the standard 11-bit format; Part B adds the extended 29-bit format. It is dense and formal — not a tutorial — but essential as a reference for edge cases and exact protocol behavior.

- **Bosch CAN 2.0 Spec (PDF)**: http://esd.cs.ucr.edu/webres/can20.pdf (UC Riverside mirror)
- **NXP-hosted version**: https://www.nxp.com/docs/en/reference-manual/BCANPSV2.pdf
- **Bosch official protocols page**: https://www.bosch-semiconductors.com/products/ip-modules/can-protocols/

The **Wikipedia CAN Bus article** (https://en.wikipedia.org/wiki/CAN_bus) is surprisingly thorough — it covers CAN history (Bosch development starting 1983, debut at SAE 1986), arbitration, physical layer with exact voltage specifications (**CANH ≈ 3.5V, CANL ≈ 1.5V dominant; both ≈ 2.5V recessive**), frame structure, error handling, and standards references. It works well as a quick-reference complement to dedicated tutorials.

### CSS Electronics — best visual CAN tutorials available

CSS Electronics produces what are arguably the clearest, most visually polished CAN Bus explainers on the internet. Their color-coded frame diagrams and infographics make complex concepts immediately graspable. The error handling tutorial is especially thorough, covering all five error detection mechanisms with practical experiments.

- **CAN Bus Explained (main intro)**: https://www.csselectronics.com/pages/can-bus-simple-intro-tutorial — Covers fundamentals, nodes, wiring, frame structure, arbitration, bit stuffing; excellent entry point
- **CAN Bus Errors Tutorial**: https://www.csselectronics.com/pages/can-bus-errors-intro-tutorial — Five error types, error frames, error counters, node state machines (Error Active/Passive/Bus Off)
- **CAN FD Explained**: https://www.csselectronics.com/pages/can-fd-flexible-data-rate-intro — Frame comparison diagrams, new bits (FDF, BRS, ESI), DLC encoding changes, compatibility considerations
- **All CAN Guides Hub**: https://www.csselectronics.com/pages/can-bus-intros-tutorials
- **160+ page Ultimate Guide (PDF)**: https://www.csselectronics.com/pages/can-bus-ultimate-guide — Free with email signup; consolidated offline reference

CSS Electronics is a CAN logger vendor, so tutorials occasionally reference their hardware, but the protocol content is vendor-neutral and high quality. **Start here for building conceptual understanding.**

### Kvaser — signal-level understanding with oscilloscope captures

Kvaser, a Swedish CAN hardware pioneer, provides multi-part tutorials that go deeper into the physical signal layer than any other free resource. Their **unique differentiator is real oscilloscope captures** showing what CAN signals actually look like on the wire, including the effects of missing termination, different bit rates, and cable length.

- **CAN Protocol Tutorial (8-part)**: https://kvaser.com/can-protocol-tutorial/ — ISO 11898 standards, message types, arbitration, physical layers, bit timing, error handling
- **CAN Physical Layers lesson**: https://kvaser.com/lesson/can-physical-layers/ — Oscilloscope images of dominant/recessive states, termination effects, real-world signal behavior
- **CAN FD Protocol Tutorial**: https://kvaser.com/can-fd-protocol-tutorial/ — Detailed technical comparison: new frame fields, DLC encoding changes, CRC differences, efficiency calculations
- **CAN FD vs Classical CAN whitepaper (PDF)**: https://kvaser.com/wp-content/uploads/2016/10/comparing-can-fd-with-classical-can.pdf — By Kent Lennartsson; efficiency tables and frame format diagrams
- **CAN Education Hub**: https://kvaser.com/about-can/can-education/
- **Bit Timing Calculator**: https://kvaser.com/support/calculators/bit-timing-calculator/

Some Kvaser content requires a free account for download. **Best resource for understanding CAN at the physical/signal level** — particularly relevant for rover hardware bus design.

### Vector Informatik — structured e-learning course

Vector, the leading automotive networking tools company, offers a free structured e-learning course with **37 learning units** (~10 minutes each) across 6 chapters. It progresses methodically from motivation and standardization through bus levels, framing, arbitration, data protection, and CAN FD. Registration required but content is free.

- **CAN E-Learning Module**: https://elearning.vector.com/mod/page/view.php?id=333
- **CAN FD chapter**: https://elearning.vector.com/mod/page/view.php?id=360
- **Physical Layer Problems App Note (PDF)**: https://cdn.vector.com/cms/content/know-how/_application-notes/AN-ANI-1-115_HS_Physical_Layer_Problems.pdf — Practical debugging: voltage checks, termination troubleshooting, ground shift problems

More formally organized than CSS Electronics or Kvaser. **Best for methodical, self-paced progression through all CAN concepts.**

### Texas Instruments application notes — physical layer gold standard

TI's application notes are the industry standard for CAN physical layer design. They provide the exact electrical specifications, cable requirements, and transceiver details needed for hardware implementation.

- **SLLA270 — CAN Physical Layer Requirements (PDF)**: https://www.ti.com/lit/an/slla270/slla270.pdf — **The definitive physical layer reference.** Covers ISO 11898-2, differential signaling, voltage levels, cable impedance (120Ω), termination resistors, maximum cable lengths, stub lengths, high-speed CAN specs (1 Mbps / 40m / 30 nodes), transceiver architecture. Written by Steve Corrigan (TI CAN applications expert). Heavily cited across the industry.
- **SLOA101 — Introduction to Controller Area Network (PDF)**: https://www.ti.com/lit/pdf/sloa101 — Higher-level protocol overview with transceiver properties, bus states, driver/receiver characteristics.

**SLLA270 is essential reading for the rover project's physical bus design** — termination, cable selection, transceiver configuration, and maximum bus length calculations.

### Additional physical layer and CAN FD resources

Several other semiconductor vendors provide excellent complementary physical layer documentation:

- **Microchip AN228 — CAN Physical Layer Discussion (PDF)**: https://ww1.microchip.com/downloads/en/appnotes/00228a.pdf — Bus voltage thresholds, ISO 11898-2 overview, transceiver behavior, slope control
- **Analog Devices AN-1123 — CAN Implementation Guide**: https://www.analog.com/en/resources/app-notes/an-1123.html — Physical bus design, isolation, signal integrity; good comparison of CAN vs RS-485
- **CopperHill Tech — Dominant and Recessive Bus Levels**: https://copperhilltech.com/blog/controller-area-network-can-bus-tutorial-dominant-and-recessive-bus-level/ — Clear explanation using transistor open-collector analogy
- **HMS Networks — CAN vs CAN FD vs CAN XL**: https://www.hms-networks.com/tech-blog/blogpost/hms-blog/2024/06/18/can-can-fd-can-xl-what-are-the-differences — Concise comparison of all three generations
- **CAN in Automation (CiA)**: https://www.can-cia.org/can-knowledge — The official CAN standards body; knowledge base, CAN FD overview at https://www.can-cia.org/can-knowledge/can-fd-the-basic-idea, and a curated book list at https://www.can-cia.org/services/can-related-books

### Books worth owning

Two books stand out for offline reference. **"A Comprehensible Guide to Controller Area Network" by Wilfried Voss** (2nd Edition, 2008, ~164 pages, ~$10 used) is widely considered the best introductory CAN book — 4.2/5 on Amazon with 90+ reviews. It covers history, frame architecture, arbitration, bit timing, error detection, fault confinement, and physical layer design, all based on ISO 11898. Available at https://copperhilltech.com/a-comprehensible-guide-to-controller-area-network/.

"Controller Area Network: Basics, Protocols, Chips and Applications" by Konrad Etschberger (2001) is more academic and authored by a CAN pioneer who developed the first CAN chip. Reviews note inconsistent English translation, making it a better secondary reference than primary learning resource.

---

## Category 2: STM32 bxCAN peripheral implementation

### ST reference manuals — the definitive register documentation

The bxCAN chapters in the ST reference manuals are the ground truth for register-level programming. Every CAN register is documented: CAN_MCR, CAN_MSR, CAN_TSR, CAN_BTR, transmit/receive mailbox registers, filter registers, and error status register.

- **RM0008 (STM32F103) — Chapter 24 "bxCAN"**: https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf — Single CAN instance, **14 filter banks**. The F103 Blue Pill's bxCAN peripheral.
- **RM0090 (STM32F4xx) — Chapter 32 "bxCAN"**: https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf — Dual CAN (CAN1 master + CAN2 slave), **28 shared filter banks**. Same bxCAN IP as F1 with dual-CAN additions.

These are essential for register-level work but are reference documents, not tutorials. Pair them with the ST Community articles below for guided walkthroughs.

### ST application notes and training materials

- **AN1077 — Overview of Enhanced bxCAN Controller (PDF)**: http://www.st.com/resource/en/application_note/cd00004224.pdf — Architectural overview of the bxCAN: block diagram, 3 TX mailboxes, transmission scheduling (ID priority vs FIFO mode), 14/28 filter banks, dual FIFO reception, Filter Match Index. **Best complement to the reference manual** — shorter and more conceptual. Covers register-level concepts.
- **STM32F7 bxCAN Product Training (PDF slides)**: https://www.st.com/resource/en/product_training/STM32F7_Peripheral_bxCAN.pdf — Slide deck covering initialization, Normal/Sleep modes, filter configuration, loopback/silent modes, interrupts, debug behavior. Applicable to all bxCAN devices including F103/F4xx. Good visual overview before diving into the reference manual.

Note: **AN3264 does not appear to exist** as a standalone bxCAN application note despite being commonly referenced online. AN1077 and the ST Community knowledge base articles serve this role instead.

### ST Community knowledge base — the modern definitive tutorials

These articles, primarily authored by ST engineer mƎALLEm, are the **single best set of STM32 CAN tutorials available** — official, authoritative, comprehensive, and free. They cover both HAL-level configuration and register-level concepts with STM32CubeMX screenshots, code examples, and GitHub project links.

- **bxCAN Loopback Mode Guide**: https://community.st.com/t5/stm32-mcus/guide-to-can-bxcan-can2-0-configuration-in-loop-back-mode-on/ta-p/771119 — **Start here.** Complete step-by-step: project creation, clock config, CAN peripheral config, filter setup, TX/RX code with interrupts, logic analyzer verification. Target: STM32F103 NUCLEO board. HAL-level with GitHub project links for multiple boards. Covers HAL_CAN_AddTxMessage, HAL_CAN_GetRxMessage, HAL_CAN_ActivateNotification, CAN_IT_RX_FIFO0_MSG_PENDING.

- **bxCAN Bit Timing Configuration**: https://community.st.com/t5/stm32-mcus/can-bxcan-bit-time-configuration-on-stm32-mcus/ta-p/689864 — **The single best bit timing resource for STM32.** Covers time quantum (tq), bit segments (BS1/BS2), sample point (87.5% recommended), CAN prescaler (BRP), APB1 clock relationship, CAN_BTR register fields. Key formulas: `tq = (BRP[9:0] + 1) × tPCLK` and `Baud rate = 1 / (tq × (1 + TS1 + TS2))`. Both HAL (CubeMX parameters) and register-level (CAN_BTR). Links to bittiming.can-wiki.info calculator.

- **bxCAN Normal Mode Guide (Part 1 — Theory)**: https://community.st.com/t5/stm32-mcus/using-can-bxcan-in-normal-mode-with-stm32-microcontrollers-part/ta-p/800502 — Hardware setup: transceivers, **120Ω termination**, GPIO pin configuration differences between F1 and other families, pin pull-up requirements, the critical detail that CAN2 requires CAN1 clock enabled. Includes Part 2 with practical implementation.

- **Dual CAN Filter Bank Explanation**: https://community.st.com/t5/stm32-mcus/stm32-in-dual-can-configuration-bxcan-filter-bank-explanation/ta-p/698739 — Explains shared filter bank allocation between CAN1 and CAN2 using the CAN2SB cursor mechanism (CAN_FMR.CAN2SB / SlaveStartFilterBank). **Solves the #1 most common dual-CAN configuration failure.** Essential for STM32F4xx with dual CAN.

- **Troubleshooting bxCAN in Loopback Mode**: https://community.st.com/t5/stm32-mcus/troubleshooting-bxcan-issues-in-loop-back-mode-on-stm32-mcus/ta-p/751884 — Common pitfalls: missing RX pin pull-up, NVIC not enabled, wrong FIFO interrupt callback, GPIO misconfiguration. **Invaluable when things don't work.**

### Bit timing calculators and tools

Configuring the CAN_BTR register (BRP, TS1, TS2, SJW) is one of the trickiest parts of CAN setup. These tools eliminate guesswork:

- **bittiming.can-wiki.info**: http://www.bittiming.can-wiki.info/ — **The gold standard.** Web calculator that specifically supports "STMicroelectronics bxCAN" as a controller type. Input APB1 clock frequency and desired baud rate; outputs BRP, TS1, TS2 values. Highlights recommended 16 TQ configurations with **87.5% sample point**. Shows register values directly. Register-level output.
- **Kvaser Bit Timing Calculator**: https://kvaser.com/support/calculators/bit-timing-calculator/ — Professional-grade, supports CAN 2.0 and CAN FD, not STM32-specific.
- **TeachMeMicro STM32 Calculator**: https://www.teachmemicro.com/stm32-can-bus-configuration-calculator/ — Article explaining manual BRP/TS1/TS2 calculation with formulas and interactive calculator.

The relationship between clocks is: **the bxCAN peripheral sits on the APB1 bus**, so its input clock is PCLK1. For STM32F103 at 72 MHz, PCLK1 is typically 36 MHz. For STM32F407 at 168 MHz, PCLK1 is typically 42 MHz. The prescaler BRP divides this clock to produce time quanta.

### ControllersTech — practical HAL tutorials with code

ControllersTech provides the most popular STM32 CAN HAL tutorial, combining blog posts with downloadable projects and companion YouTube videos.

- **CAN Protocol in STM32 (bxCAN Normal Mode)**: https://controllerstech.com/can-protocol-in-stm32/ — Full tutorial: CubeMX configuration, MCP2551 transceiver wiring, filter configuration (Mask mode, 32-bit scale), TX code with CAN_TxHeaderTypeDef, RX with interrupts using HAL_CAN_RxFifo0MsgPendingCallback. Covers the **FilterIdHigh bit shifting (<<5 for StdID)** that trips up many developers. Targets STM32F103/F407. HAL-level. Last updated December 2025.
- **CAN Bus Tutorials Index**: https://controllerstech.com/stm32-hal/peripherals/can-bus/

### CAN filter configuration — the notoriously confusing topic

Filter bank configuration is where most developers get stuck. These resources specifically address the confusion:

- **Max Schulz — STM32 CAN Identifier Filtering**: https://schulz-m.github.io/2017/03/23/stm32-can-id-filter/ — **One of the best filter explanations available.** Covers 32-bit vs 16-bit filter modes, Mask mode vs Identifier mode, bit shifting for FilterIdHigh/FilterIdLow/FilterMaskIdHigh/FilterMaskIdLow. Register-level with HAL struct mapping. Targets STM32F4xx.
- **ST Community filter thread**: https://community.st.com/t5/stm32-mcus-products/can-filter-configuration/td-p/465853 — Community discussion covering 28 filter banks, modes, and CAN1/CAN2 splitting.

### Register-level and bare-metal CAN code

For developers who prefer working below the HAL:

- **OpenCyphal/UAVCAN bxCAN Driver**: https://github.com/UAVCAN/platform_specific_components/blob/master/stm32/libcanard/bxcan/src/bxcan.c — **Best register-level CAN reference code available.** Production-quality bare-metal bxCAN driver handling dual CAN, filter bank splitting, TX deadline management, and error handling. MIT license. Extensive comments explaining filter bank allocation for single vs dual bxCAN configurations.

- **libopencm3 CAN Implementation**: Source at https://github.com/libopencm3/libopencm3/blob/master/lib/stm32/can.c, header at https://github.com/libopencm3/libopencm3/blob/master/include/libopencm3/stm32/can.h, example at https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/obldc-strip/can/can.c — Thin abstraction over registers; you can see CAN_MCR, CAN_MSR, CAN_TSR definitions directly. LGPL license. Supports STM32F1/F4.

- **rhye.org — STM32 & OpenCM3 Part 3: CANBus**: https://rhye.org/post/stm32-with-opencm3-3-canbus/ — **Best written register-level CAN tutorial.** Comprehensive walkthrough using libopencm3 covering bus termination, message sending/receiving, and filter bank configuration with practical code. Companion GitHub repo with KiCad hardware files at https://github.com/rschlaikjer/hello-stm32-3-canbus.

- **nopnop2002/Arduino-STM32-CAN**: https://github.com/nopnop2002/Arduino-STM32-CAN — Despite being Arduino-based, contains **deep register-level explanations** of TX mailboxes, CAN_IER, CAN_MCR, CAN_TSR. Supports F103, F303, F405, F407, F446, F767. Includes Raspberry Pi + MCP2515 SocketCAN integration. Excellent README documentation.

- **Keil STM32F103 CAN Example**: https://www.keil.com/download/docs/351.asp — Legacy register-level CAN example at 500 kbit/s sending ADC values. Dated but still a useful reference.

### HAL-level code examples and wikis

- **STM32World Wiki — CAN Tutorial**: https://stm32world.com/wiki/STM32_CAN — Complete loopback example with filter setup, HAL code, and link to GitHub source. STM32F405 at APB1 42 MHz. Companion GitHub repo: https://github.com/STM32World/firmware/tree/master/mcustm32f405_can_loopback
- **timsonater/stm32-CAN-bus-example-HAL-API**: https://github.com/timsonater/stm32-CAN-bus-example-HAL-API — Clean, well-commented HAL example for STM32F4. Companion YouTube video at https://www.youtube.com/watch?v=ymD3F0h-ilE
- **Official ST CAN Loopback Example (STM32CubeF4)**: https://github.com/STMicroelectronics/STM32CubeF4/blob/master/Projects/STM32469I_EVAL/Examples/CAN/CAN_Loopback/readme.txt — Official ST example code
- **DailyDuino — STM32 CAN Bus Tutorial**: https://www.dailyduino.com/index.php/2020/06/01/stm32-can-bus/ — Blue Pill specific, with CubeMX screenshots and baud rate calculation walkthrough. Companion GitHub: https://github.com/DailyDuino/STM32-CAN-bus-example-rx-tx
- **Matthew Tran — Setting Up CAN on STM32**: https://matthewtran.dev/2020/01/setting-up-the-can-bus-on-stm32/ — Compares Mbed and STM32CubeIDE approaches; clear APB1 clock relationship explanation

### Jetson Orin Nano integration resources

Connecting your STM32 CAN network to the Jetson Orin Nano requires bridging CAN to Linux SocketCAN. Note that **the STM32F103 cannot use USB and CAN simultaneously** (they share SRAM), so a dedicated USB-CAN bridge chip or separate adapter is needed.

- **NVIDIA Jetson CAN Documentation**: https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html — Official docs covering kernel driver loading, pinmux configuration, SocketCAN setup, loopback testing, CAN FD support up to 15 Mbps on Jetson Orin platforms.
- **candleLight Firmware (USB-CAN)**: https://github.com/candle-usb/candleLight_fw — gs_usb-compatible firmware for USB-CAN adapters. Works natively with Linux SocketCAN without slcand. Supports STM32F042/F072/G0B1 targets. **The go-to open-source USB-CAN firmware for Linux.**
- **CANable Getting Started**: https://canable.io/getting-started.html — Complete guide for USB-CAN adapter setup with SocketCAN, including slcand and candlelight firmware modes, python-can integration. Works on ARM Linux platforms.
- **PragmaticLinux SocketCAN auto-setup**: https://www.pragmaticlinux.com/2021/07/automatically-bring-up-a-socketcan-interface-on-boot/ — systemd-networkd configuration for persistent CAN interfaces. Uses PCAN-USB with STM32 test node.
- **SocketCAN on Linux setup guide**: https://dokkev.github.io/socketcan/ — Practical guide with udev rules for persistent device naming.

### CAN network design for the rover project

- **JCOM1939 — CAN Bus Topology and Network Design**: https://jcom1939.com/can-bus-topology-and-network-design/ — Deep dive into bus topology, stub lengths, termination, signal integrity at different speeds, CAN FD topology constraints
- **ThinkRobotics — CAN Bus in Robotics Projects**: https://thinkrobotics.com/blogs/learn/using-can-bus-in-robotics-projects-a-comprehensive-guide — Message structure, ID allocation, multi-protocol gateways (CAN ↔ Ethernet), time synchronization for robotic systems
- **ON Semiconductor AND8351 — Multiple CAN Bus Network (PDF)**: https://www.onsemi.com/pub/Collateral/AND8351-D.PDF — Multi-bus network design with signal delay analysis and node addition strategies

Key design parameters for the rover: at **500 kbit/s** max bus length is ~100m, at **1 Mbit/s** max ~40m. Keep stubs under 0.3m at 1 Mbit/s. Use shielded twisted pair with ~120Ω impedance and **120Ω termination resistors at each end** (measure 60Ω between CANH/CANL when bus is powered down). Classical CAN with 8-byte payloads likely suffices unless high-bandwidth sensors demand CAN FD's 64-byte frames.

### YouTube channels as secondary resources

- **ControllersTech** (https://www.youtube.com/@ControllersTech) — **Best video resource for STM32 CAN.** Complete tutorial series covering bxCAN and FDCAN setup with companion blog posts and downloadable code.
- **STM32World** (https://www.youtube.com/watch?v=RrR-BDoBoSg) — CAN loopback and real bus tutorials with companion wiki.
- **Phil's Lab** (https://www.youtube.com/@PhilsLab) — Excellent for STM32 PCB design and firmware fundamentals (SPI, USB, SWD). No dedicated CAN video found, but highly recommended for designing CAN transceiver circuits on custom PCBs.
- **timsonater CAN video** (https://www.youtube.com/watch?v=ymD3F0h-ilE) — Practical two-board CAN communication demo with companion GitHub repo.

## Conclusion

The CAN Bus ecosystem is remarkably well-documented for a 40-year-old protocol. For **conceptual grounding**, start with CSS Electronics' visual guides, progress through Kvaser's oscilloscope-level tutorials, and keep TI's SLLA270 as your physical layer bible. For **STM32 implementation**, the five ST Community knowledge base articles (loopback → bit timing → normal mode → filters → troubleshooting) provide the most complete guided path from zero to working CAN bus. **Register-level developers** should study the OpenCyphal bxCAN driver alongside RM0008/RM0090 chapter by chapter, using bittiming.can-wiki.info to compute CAN_BTR values. For the **Jetson integration**, plan around a candleLight-compatible USB-CAN adapter running gs_usb firmware — it provides native SocketCAN support without custom drivers. One critical hardware note: the STM32F103 shares USB and CAN SRAM, so you cannot use both peripherals simultaneously on the Blue Pill — plan your rover's bridge node accordingly.
