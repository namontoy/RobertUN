# Jetson Adapter + Ethernet Troubleshooting

This problem happens when the Jetson, the USB-to-Ethernet adapter, and the PC do not keep a stable Ethernet link. In practice, the current SSH or adapter-based session drops, the network connection starts reconnecting in a loop, and the Ethernet link blinks on and off. The usual meaning is that one of these layers is failing: the adapter is losing power, the kernel is failing to initialize the Ethernet hardware correctly, or the PC and Jetson are not agreeing on IP configuration. The result is that the cable is physically connected, but the link is not stable enough to keep a normal session alive.

## 1. How to Check What Is Failing

- `lsusb`
  Checks the physical layer. If the Realtek/Dell adapter does not appear, the adapter has no power or the USB port is failing.

- `sudo dmesg -w | grep eth`
  Checks the hardware-to-kernel layer. If you see `Get ether addr fail`, the adapter is crashing before normal networking can start.

- `nmcli device status`
  Checks the OS layer. If it shows `connecting (getting IP configuration)`, the hardware is awake but the PC and Jetson are not negotiating correctly.

- `arp -a`
  Checks whether both sides are actually responding to each other. If you see `<incomplete>`, the PC sees something on the wire but is not getting a proper reply.

## 2. Commands That Worked

### Stop the flapping

```bash
sudo nmcli connection modify jetson-direct ipv4.method manual
```

This disables DHCP for that profile and stops repeated reconnect/reset behavior.

### Put the PC on the same subnet

```bash
sudo nmcli connection modify jetson-direct ipv4.addresses 10.42.0.1/24
```

This places the PC in the same `10.42.0.x` network as a Jetson at `10.42.0.110`.

### Apply the profile

```bash
sudo nmcli connection up jetson-direct
```

This restarts the connection with the manual settings.

## 3. Future Recovery Protocol

1. Check LEDs on the Ethernet port.
   No lights usually means power or hardware trouble. Repeated blinking often means DHCP flapping.

2. Check `arp -a`.
   If you see `<incomplete>`, the addressing is probably wrong.

3. Try the fail-safe mode if manual does not work:

```bash
sudo nmcli connection modify jetson-direct ipv4.method shared
```

This makes the PC manage the link more aggressively and is often more stable for a direct PC-to-Jetson cable.

## Checklist

- Hardware: `lsusb`, `dmesg`, port LEDs
- Addressing: `ip addr`, `nmcli`, same subnet
- Stability: use `manual` or `shared`, not automatic DHCP
