## Troubleshooting Jetson Nano SSH Issues over Direct Ethernet

When connecting to a Jetson Nano over a direct Ethernet connection, SSH may fail even if the Ethernet cable is connected and the interface appears as active. In some cases, the problem is not the SSH credentials, but the fact that the Jetson Nano has not received a valid IP address through DHCP.

This usually happens when the host computer has a static/manual Ethernet configuration but is not sharing the connection or acting as a DHCP server. As a result, the Jetson Nano keeps requesting an IP address, but no address is assigned. Since the Jetson does not have a reachable IP, SSH cannot reach port `22`.

Typical symptoms include:

```bash
ssh username@<jetson-ip>
```

Resulting in errors such as:

```bash
Connection timed out
No route to host
Destination Host Unreachable
Permission denied
```

Before assuming the password or SSH service is wrong, first verify that the Jetson Nano is actually receiving an IP address.

---

## 1. Check the Ethernet Connection

List the active network devices:

```bash
nmcli device status
```

Identify the Ethernet interface being used for the Jetson connection. It may look similar to:

```bash
enx0c379654d040  ethernet  connected  jetson-direct
```

Then check the IP configuration of that interface:

```bash
ip addr show <ethernet-interface>
```

Example:

```bash
ip addr show enx0c379654d040
```

The interface should be up and connected.

---

## 2. Check Whether the Jetson Received an IP Address

Use:

```bash
ip neigh
```

If the Jetson received an IP address, you should see an entry similar to:

```bash
10.42.0.xxx dev enx0c379654d040 lladdr 3c:6d:66:61:da:f1 REACHABLE
```

If no Jetson-related IP appears, or if the entry appears as:

```bash
FAILED
```

then the Jetson is not currently reachable.

You can also check NetworkManager DHCP lease files:

```bash
sudo ls -la /var/lib/NetworkManager/ | grep dnsmasq
```

Then read the lease file for the Ethernet interface:

```bash
sudo cat /var/lib/NetworkManager/dnsmasq-<ethernet-interface>.leases
```

Example:

```bash
sudo cat /var/lib/NetworkManager/dnsmasq-enx0c379654d040.leases
```

A valid lease may look like this:

```bash
1776551622 3c:6d:66:61:da:f1 10.42.0.110 ingfisica-desktop 01:3c:6d:66:61:da:f1
```

The important parts are:

```bash
3c:6d:66:61:da:f1
10.42.0.110
ingfisica-desktop
```

This indicates that the Jetson was assigned an IP address.

However, be careful: the lease file may contain an old IP address. Always confirm that the IP is reachable:

```bash
ping -c 4 10.42.0.110
```

If the ping fails, the lease is probably outdated.

---

## 3. Check Whether the Jetson Is Asking for DHCP

If no current IP address appears, use `tcpdump` to check whether the Jetson is requesting an IP address:

```bash
sudo tcpdump -i <ethernet-interface> -n -e "arp or port 67 or port 68"
```

Example:

```bash
sudo tcpdump -i enx0c379654d040 -n -e "arp or port 67 or port 68"
```

Then restart or reconnect the Jetson Nano.

If the Jetson is asking for an IP address, you should see DHCP traffic like this:

```bash
0.0.0.0.68 > 255.255.255.255.67: BOOTP/DHCP, Request from 3c:6d:66:61:da:f1
```

This means the Jetson is connected and requesting an IP address, but the host computer may not be responding with a DHCP offer.

To stop `tcpdump`, press:

```bash
Ctrl + C
```

---

## 4. Check the NetworkManager Configuration

Check whether the Ethernet connection is configured as `manual` or `shared`:

```bash
nmcli connection show jetson-direct | grep -E "connection.interface-name|ipv4.method|ipv4.addresses|ipv6.method"
```

Example output with a problem:

```bash
connection.interface-name:              enx0c379654d040
ipv4.method:                            manual
ipv4.addresses:                         10.42.0.1/24
ipv6.method:                            ignore
```

The issue is:

```bash
ipv4.method: manual
```

With `manual`, the host has its own static Ethernet configuration, but it does not provide DHCP to the Jetson Nano.

For direct Ethernet access, the connection should usually be configured as:

```bash
ipv4.method: shared
```

This allows the host computer to act as a DHCP server and assign an IP address to the Jetson Nano.

---

## 5. Fix the DHCP Sharing Problem

Change the connection to shared mode:

```bash
nmcli connection modify jetson-direct ipv4.method shared ipv4.addresses 10.42.0.1/24 ipv6.method ignore
nmcli connection down jetson-direct
nmcli connection up jetson-direct
```

Then verify the configuration:

```bash
nmcli connection show jetson-direct | grep -E "connection.interface-name|ipv4.method|ipv4.addresses|ipv6.method"
```

Expected result:

```bash
connection.interface-name:              enx0c379654d040
ipv4.method:                            shared
ipv4.addresses:                         10.42.0.1/24
ipv6.method:                            ignore
```

---

## 6. Check the New Jetson IP Address

After enabling shared mode, check the neighbor table again:

```bash
ip neigh
```

Also check the DHCP lease file:

```bash
sudo cat /var/lib/NetworkManager/dnsmasq-<ethernet-interface>.leases
```

Example:

```bash
sudo cat /var/lib/NetworkManager/dnsmasq-enx0c379654d040.leases
```

If the Jetson received an IP, test it:

```bash
ping -c 4 <jetson-ip>
```

Example:

```bash
ping -c 4 10.42.0.110
```

If the ping works, connect using SSH:

```bash
ssh username@<jetson-ip>
```

Example:

```bash
ssh ingfisica@10.42.0.110
```

---

## 7. Optional: Restart NetworkManager

If the connection still does not assign an IP address, restart NetworkManager:

```bash
sudo systemctl restart NetworkManager
nmcli connection up jetson-direct
```

Then check again:

```bash
ip neigh
sudo cat /var/lib/NetworkManager/dnsmasq-<ethernet-interface>.leases
```

---

## Summary

If SSH cannot reach the Jetson Nano over direct Ethernet, first verify that the Jetson has received a valid IP address. A common cause is that the Ethernet connection is configured as `manual` instead of `shared`.

Use `tcpdump` to confirm whether the Jetson is sending DHCP requests. If DHCP requests are visible but no IP address is assigned, change the NetworkManager connection to shared mode:

```bash
nmcli connection modify jetson-direct ipv4.method shared ipv4.addresses 10.42.0.1/24 ipv6.method ignore
nmcli connection down jetson-direct
nmcli connection up jetson-direct
```

Once the Jetson receives an IP address, test connectivity with `ping`, then connect using SSH.
