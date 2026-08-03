# SEARCH – Better Slow Start for TCP

**SEARCH** is an enhancement to TCP and QUIC that exits the slow start phase *after* the congestion point is reached but *before* inducing unnecessary packet loss.

---

🔧 Kernel Version Compatibility

This main branch contains the implementation of SEARCH, designed for Linux kernel 6.13.

## 📘 Overview

TCP slow start ramps up the congestion window (cwnd) exponentially until reaching the congestion point. However:

- **TCP Cubic + HyStart (default Linux)** often *exits slow start too early*, reducing throughput.
- **TCP without HyStart** often *exits too late*, causing packet loss.

To improve performance, we developed **SEARCH — Slow start Exit At Right CHokepoint**, which:

- Estimates the congestion point based on delivered vs. sent bytes.
- Smooths estimates to handle latency variation.
- Normalizes behavior for different link capacities.

### ✔ Proven Across Diverse Networks
Extensive evaluations over **4G LTE**, **LEO**, **GEO satellite**, and **Wi-Fi** show:

- **Earlier, correct exit** from slow start
- **Higher throughput** than HyStart-on
- **Lower packet loss** than HyStart-off

---
## Version

### SEARCH 3.1

uses only delivered bytes

sets bin values based on cumulative delivered bytes

reduces bits in bin array with scale factor

resets the algorithm if several missed bins

resets algorithm if app limited

### SEARCH 4.0

uses sent and delivered bytes

sets bin values based on cumulative bytes

reduces bits in bin array with scale factor

Upon exit, drain built-up queuing to target cwnd

resets algorithm if app limited

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

* ⚠️ Additional Kernel Modification (Required for SEARCH 4.0)

  If you are using SEARCH version 4.0, you must increase the TCP congestion control private data size in the Linux kernel.

  ** Modify inet_connection_sock

  Edit the following file:

	```bash
	include/net/inet_connection_sock.h
	```
	Locate this line:
	
	```bash
	u64 icsk_ca_priv[104 / sizeof(u64)];
	```
	
	and change it to:
	
	```bash
	u64 icsk_ca_priv[200 / sizeof(u64)];
	```

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
## SEARCH Parameters

* Set cwnd at Exit Time (SEARCH 3.1 only)

	In SEARCH version 3.1, you can control whether the congestion window is rolled back at slow start exit.
	
	Enable
	
	```bash
	sudo sh -c "echo '1' > /sys/module/tcp_cubic_search/parameters/cwnd_rollback"
	```
	
	Disable
	
	```bash
	sudo sh -c "echo '0' > /sys/module/tcp_cubic_search/parameters/cwnd_rollback"
	```

 * search_alpha (SEARCH 3.1 only)

	The search_alpha parameter controls the sensitivity of SEARCH to missed bins, which determines when the algorithm resets.

	```bash
	sudo sh -c "echo '<value>' > /sys/module/tcp_cubic_search/parameters/search_alpha"
 	```

 	Replace <value> with your desired integer.

	Default = 2 → corresponds to ~2 RTTs tolerance for missed bins

	Lower value → more aggressive reset

	Higher value → more tolerant (less frequent resets)

* Drain Phase (SEARCH 4.0)

	In SEARCH version 4.0, cwnd rollback is replaced with a drain phase, which gradually reduces the built-up queue after finding capacity.
	
	The drain behavior is controlled directly in the code:

	```bash
	#define SEARCH_DRAIN_ACKEDSEG_THRESH 3  /* ACKed-segment threshold to permit CWND increase during drain */
	```
 
	You can adjust this value to control how aggressively the queue is drained:
	
	Lower value → faster cwnd growth during drain (less aggressive draining)

	Higher value → slower cwnd growth (more aggressive draining)
