# SEARCH – Better Slow Start for TCP

**SEARCH** (*Slow start Exit At Right CHokepoint*) is an enhancement to TCP slow start that aims to exit near the congestion point, avoiding both premature exit and excessive queue buildup or packet loss.

---

## Kernel Version Compatibility

This branch contains a SEARCH implementation targeting **Linux kernel 6.13**. The source file for version 4.1 is `tcp_cubic_search.c`.

**Note:** These instructions assume the congestion-control algorithm name in `tcp_cubic_search.c` has been changed to **`cubic_search`** (in the `tcp_congestion_ops` `.name` field). The module name is based on the source filename and build configuration.

## Overview

TCP slow start exponentially increases the congestion window (`cwnd`) until it exits slow start. However:

- **TCP CUBIC with HyStart** can exit too early, leaving available bandwidth unused.
- **TCP without HyStart** can exit too late, causing excessive queueing and packet loss.

SEARCH addresses these issues by:

- Comparing cumulative **sent** and **delivered** bytes over time windows to detect the congestion point.
- Using time bins and scaling to support a range of path capacities and RTTs.
- Protecting the exit decision from **application-limited delivery-rate samples**.
- Using a **drain phase** to reduce excess in-flight data after detection, when needed.

### Evaluated Across Diverse Networks

Prior SEARCH evaluations across **4G LTE**, **LEO and GEO satellite**, and **Wi-Fi** have investigated slow-start exit timing, throughput, and packet loss. The features below describe the **version 4.1 implementation** and should not be interpreted as independent performance claims for this version.

---
### SEARCH 4.1

Includes SEARCH 4.0 functionality and adds/refines:

1. **App-limited bin protection:** Tracks app-limited status across the 10 SEARCH delivery bins. A bin is marked app-limited if a valid delivery-rate sample within that bin has `tp->rate_app_limited` set. SEARCH blocks the exit/drain decision when the number of marked bins reaches the configured threshold.
2. **Configurable app-limited threshold:** Adds `search_app_limited_bin_thresh` (default **1**, range **0–10**). A value of **0** disables this protection.
3. **1.25× drain target:** Sets the drain target to **1.25 × estimated BDP**, subject to a minimum initial congestion window (`TCP_INIT_CWND`). The margin reduces the risk of draining below the available capacity estimate.
4. **Conditional drain:** Enters drain only when `target_cwnd < current_cwnd` **and** `inflight > target_cwnd`. Otherwise, SEARCH exits slow start directly at the current `cwnd`.
5. **Drain growth control:** Uses `SEARCH_DRAIN_ACKEDSEG_THRESH = 16` as the default threshold for congestion-window increments during drain.

---

## Build

To integrate SEARCH TCP into a compatible Linux kernel source tree:

1. Copy `tcp_cubic_search.c` to `net/ipv4/`.

2. Add an entry to `net/ipv4/Kconfig`:

   ```text
   config TCP_CONG_SEARCH
       tristate "SEARCH TCP"
       default n
       help
         SEARCH TCP implements a delivered-versus-sent byte-based
         mechanism for detecting the slow-start exit point, with
         app-limited exit protection and an optional drain phase.
   ```

3. Add the object to `net/ipv4/Makefile`:

   ```makefile
   obj-$(CONFIG_TCP_CONG_SEARCH) += tcp_cubic_search.o
   ```

4. **Increase congestion-control private storage (required for the SEARCH state).** In `include/net/inet_connection_sock.h`, replace the default allocation:

   ```c
   u64 icsk_ca_priv[104 / sizeof(u64)];
   ```

   with a sufficiently large allocation, for example:

   ```c
   u64 icsk_ca_priv[256 / sizeof(u64)];
   ```

   **Important:** Confirm this allocation fits `sizeof(struct bictcp)` on the target kernel; the module includes a `BUILD_BUG_ON(sizeof(struct bictcp) > ICSK_CA_PRIV_SIZE)` check. Rebuild against the modified kernel headers. The earlier README's 200-byte value is not assumed sufficient for 4.1.

5. Enable `CONFIG_TCP_CONG_SEARCH=m` (or `y`) in the kernel configuration, then build and install using the standard kernel build procedure for your environment:

   ```bash
   make -j"$(nproc)"
   sudo make modules_install
   sudo make install
   ```

   Reboot into the modified kernel if necessary. Module/BTF registration requirements may depend on your kernel configuration.

---

## Helpful Commands

Check available congestion-control algorithms:

```bash
sysctl net.ipv4.tcp_available_congestion_control
```

Check the active congestion-control algorithm:

```bash
sysctl net.ipv4.tcp_congestion_control
```

Set SEARCH as the active congestion-control algorithm:

```bash
sudo sysctl -w net.ipv4.tcp_congestion_control=cubic_search
```

**Module parameters:** The commands below assume the compiled kernel module is named `tcp_cubic_search` (as with `tcp_cubic_search.ko`). If a different module name was used, update the `/sys/module/.../parameters/` path accordingly.

### Slow-start modes

Enable SEARCH:

```bash
sudo sh -c "echo 1 > /sys/module/tcp_cubic_search/parameters/slow_start_mode"
```

Enable HyStart:

```bash
sudo sh -c "echo 2 > /sys/module/tcp_cubic_search/parameters/slow_start_mode"
```

Disable both (standard slow start):

```bash
sudo sh -c "echo 0 > /sys/module/tcp_cubic_search/parameters/slow_start_mode"
```

---

## SEARCH Parameters (Version 4.1)

| Parameter | Default | Description |
| --- | ---: | --- |
| `slow_start_mode` | `1` | `0`: standard slow start; `1`: SEARCH; `2`: HyStart |
| `search_window_duration_factor` | `35` | Sets window duration to approximately **3.5 × initial RTT** |
| `search_thresh` | `26` | Fixed normalized deficit threshold (%) for SEARCH exit detection |
| `search_app_limited_bin_thresh` | `1` | Number of app-limited bins (among 10) needed to block exit; `0` disables protection |
| `SEARCH_DRAIN_FACTOR` | `125` | Compile-time drain margin of **1.25×** the estimated BDP |
| `SEARCH_DRAIN_ACKEDSEG_THRESH` | `16` | Compile-time acknowledged-segment threshold for a cwnd increment during drain |

### App-limited protection (new in SEARCH 4.1)

SEARCH uses the Linux TCP delivery-rate sample's `tp->rate_app_limited` flag, accepting it only when `tp->rate_interval_us > 0` and `tp->rate_delivered > 0`. If any such sample in a bin is marked app-limited, that bin is flagged.

Before a detected slow-start exit, SEARCH counts app-limited bins in its **10-bin delivered-data window**. With the default threshold of **1**, any flagged bin blocks the exit/drain decision; SEARCH continues collecting samples and can reconsider on a later evaluation.

Change the threshold at runtime:

```bash
# Default: block exit if at least one of the 10 bins is app-limited
sudo sh -c "echo 1 > /sys/module/tcp_cubic_search/parameters/search_app_limited_bin_thresh"

# Disable app-limited protection
sudo sh -c "echo 0 > /sys/module/tcp_cubic_search/parameters/search_app_limited_bin_thresh"
```

Thresholds from `1` through `10` are supported; the implementation clamps out-of-range values to `0–10` during evaluation.

### Drain phase (updated in SEARCH 4.1)

After the SEARCH exit condition passes the app-limited check, the implementation estimates BDP from delivered bytes over approximately one RTT. It then calculates:

```text
target_cwnd = max(1.25 × estimated_BDP_in_segments, TCP_INIT_CWND)
```

SEARCH enters drain only when both conditions hold:

```text
target_cwnd < current_cwnd
inflight > target_cwnd
```

If either condition is false, SEARCH exits slow start directly. During drain, `cwnd` is adjusted toward the target, with increments controlled by the count of acknowledged segments. Upon reaching the target, SEARCH sets `ssthresh` to the target and resets its SEARCH state.

The drain parameters are compile-time constants:

```c
#define SEARCH_DRAIN_FACTOR 125
#define SEARCH_DRAIN_ACKEDSEG_THRESH 16
```

- **Lower `SEARCH_DRAIN_ACKEDSEG_THRESH`:** Allows faster cwnd increase during drain (less aggressive draining).
- **Higher `SEARCH_DRAIN_ACKEDSEG_THRESH`:** Limits cwnd increase more strongly (more aggressive draining).

To change either constant, edit the source and rebuild the module/kernel.

---
