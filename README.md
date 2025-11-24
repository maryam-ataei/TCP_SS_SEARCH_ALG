# SEARCH – Better Slow Start for TCP

**SEARCH** is an enhancement to TCP and QUIC that exits the slow start phase *after* the congestion point is reached but *before* inducing unnecessary packet loss.

---

🔧 Kernel Version Compatibility

This main branch contains the latest implementation of SEARCH 3.1, designed for Linux kernel 6.13.

## 📘 Overview

TCP slow start ramps up the congestion window (cwnd) exponentially until reaching the congestion point. However:

- **TCP Cubic + HyStart (default Linux)** often *exits slow start too early*, reducing throughput.
- **TCP without HyStart** often *exits too late*, causing packet loss.

To improve performance, we developed **SEARCH — Slow start Exit At Right CHokepoint**, which:

- Estimates the congestion point based on delivered vs. expected bytes.
- Smooths estimates to handle latency variation.
- Normalizes behavior for different link capacities.

### ✔ Proven Across Diverse Networks
Extensive evaluations over **4G LTE**, **LEO**, **GEO satellite**, and **Wi-Fi** show:

- **Earlier, correct exit** from slow start
- **Higher throughput** than HyStart-on
- **Lower packet loss** than HyStart-off

---

## ⚙ Search Options (General)

- On exit, optionally lower cwnd to value from 2 RTTs prior  

---

## 🔁 Reset Behavior in SEARCH 3.1 (Missed-Bin Logic)

search_alpha is a sensitivity parameter that determines how many missed bins SEARCH tolerates before triggering a reset.

✔ Default Setting (Reset Disabled for Missed Bins)

In this implementation of SEARCH 3.1, search_alpha is set to a very large value, effectively disabling automatic resets based on missed bins.

🔧 Enabling Reset Behavior

If missed-bin resetting is desired, users can set search_alpha to a smaller value. A smaller threshold makes SEARCH more responsive to stalled bin progression.

---

## Build

Follow these steps to integrate SEARCH TCP into your kernel:

* Add `tcp_cubic_search.c` file to `/net/ipv4/`

* Modify `net/ipv4/Kconfig` to include the SEARCH TCP configuration:
	  
	  config TCP_CONG_SEARCH
		tristate "SEARCH TCP"
		default n
		help
		   SEARCH TCP congestion control implements a search mechanism to dynamically adjust
  		   the congestion control state based on the observed network conditions. The algorithm
  		   divides time into bins and analyzes the sum total of delivered bytes within these bins
  		   to decide when to exit the slow start state and enter the congestion avoidance state.
  		  This decision is based on comparing the current delivered bytes to the delivered bytes
  		  one round-trip time ago and when the delivered bytes no longer increase, the capacity
  		  chokepoint has been detected.  Upon detection, SEARCH transitions the congestion control
  		  state from slow start to congestion avoidance.

* Add the `.o` file to `net/ipv4/Makefile`
  
  the line should look like: `obj-$(CONFIG_TCP_CONG_SEARCH) += tcp_cubic_search.o`
  
* Run the following commands:

    ```bash
    sudo make
    sudo make modules_install
    sudo make install
    ```

## Helpful Commands

Check available congestion control algs:

```bash
sysctl net.ipv4.tcp_available_congestion_control
```

Check current congestion control alg:

```bash
sysctl net.ipv4.tcp_congestion_control
```

Set current congestion control alg:

```bash
sudo sysctl -w net.ipv4.tcp_congestion_control=cubic_search
```
    
Managing slow start mode

Enable SEARCH

```bash
sudo sh -c "echo '1' > /sys/module/tcp_cubic_search/parameters/slow_start_mode"
```

Enable HyStart

```bash
sudo sh -c "echo '2' > /sys/module/tcp_cubic_search/parameters/slow_start_mode"
```

Disable both

```bash
sudo sh -c "echo '0' > /sys/module/tcp_cubic_search/parameters/slow_start_mode"
```
---

Set cwnd at Exit Time

Enable

```bash
sudo sh -c "echo '1' > /sys/module/tcp_cubic_search/parameters/cwnd_rollback"
```

Disable

```bash
sudo sh -c "echo '0' > /sys/module/tcp_cubic_search/parameters/cwnd_rollback"
```
