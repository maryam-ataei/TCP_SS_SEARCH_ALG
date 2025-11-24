# SEARCH – Better Slow Start for TCP

**SEARCH** is an enhancement to TCP and QUIC that exits the slow start phase *after* the congestion point is reached but *before* inducing unnecessary packet loss.

---

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

## 🔢 Versions of the SEARCH Algorithm

### **SEARCH 1.0**
- Uses sent + delivered bytes  
- Bins based on deltas from previous bins

### **SEARCH 2.0**
- Uses only delivered bytes  
- Bins based on deltas

### **SEARCH 3.0**
- Uses only delivered bytes  
- Bins based on *cumulative* delivered bytes

### **SEARCH 3.1**
- Applies bin array scale-factor reduction  
- Resets algorithm on:
  - several missed bins  
  - app-limited state  
- *Does not require memory changes from the kernel*

---

## ⚙ Search Options (General)

- Disable interpolation when previous window falls between bins  
- On exit, optionally lower cwnd to value from 2 RTTs prior  
- Adjust window size when bins are insufficient

---

## 🧩 Dependency — Kernel Modification

The kernel needs to be recompiled with a larger `ICSK_CA_PRIV_SIZE`.
Edit:

`include/net/inet_connection_sock.h`

and for these lines:

````
  u64                       icsk_ca_priv[104 / sizeof(u64)];
  
  #define ICSK_CA_PRIV_SIZE      (13 * sizeof(u64))
````

change the number `104` to `200` and `13` to `25`.  Then rebuild and reboot the kernel normally.


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

	sysctl net.ipv4.tcp_available_congestion_control

Check current congestion control alg:

	sysctl net.ipv4.tcp_congestion_control

Set current congestion control alg:

	sudo sysctl -w net.ipv4.tcp_congestion_control=cubic_search
    
	
Managing HyStart functionality(v2 → v3.0):

	Disable hystart: 
 
 		sudo sh -c "echo '0' > /sys/module/tcp_cubic_search/parameters/hystart"
   
 	Enable hystart: 
  
  		sudo sh -c "echo '1' > /sys/module/tcp_cubic_search/parameters/hystart"
      
Managing SEARCH (v2 → v3.0)

Disable SEARCH:

    ```bash
sudo sh -c "echo '0' > /sys/module/tcp_cubic_search/parameters/search"
    ```

Enable SEARCH:

    ```bash
sudo sh -c "echo '1' > /sys/module/tcp_cubic_search/parameters/search"
    ```
	
Managing SEARCH (v3.1)

Enable SEARCH:

    ```bash
  sudo sh -c "echo '1' > /sys/module/your_module_name/parameters/slow_start_mode"
    ```
	
Enable HyStart:

    ```bash
  sudo sh -c "echo '2' > /sys/module/cubic_with_search/parameters/slow_start_mode"
    ```
Disable both:

    ```bash
  sudo sh -c "echo '0' > /sys/module/cubic_with_search/parameters/slow_start_mode"
    ```
	
Set cwnd at Exit Time

Enable:

    ```bash
sudo sh -c "echo '1' > /sys/module/tcp_cubic_search/parameters/cwnd_rollback"
    ```
Disable:

    ```bash
sudo sh -c "echo '0' > /sys/module/tcp_cubic_search/parameters/cwnd_rollback"
    ```
	
Interpolation (v3.0 and older)

Enable:
    ```bash
sudo sh -c "echo '1' > /sys/module/tcp_cubic_search/parameters/do_intpld"
    ```

Disable:
    ```bash
sudo sh -c "echo '0' > /sys/module/tcp_cubic_search/parameters/do_intpld"
    ```
