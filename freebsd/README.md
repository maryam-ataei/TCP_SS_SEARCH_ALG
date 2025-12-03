# FreeBSD with TCP SEARCH

This repository includes the implementation of **TCP SEARCH (Slow start Exit At Right CHokepoint)** in FreeBSD.

## Files Included

- `cc_newreno_search.c`  
- `cc_newreno_search.h`  

These files implement the SEARCH enhancement on top of NewReno congestion control.

## Base Version

The code is based on **FreeBSD 14.2**.

## Purpose

TCP SEARCH improves slow start behavior by exiting at the appropriate chokepoint to improve network performance.

Learn more about the SEARCH algorithm here: [https://search-ss.wpi.edu/](https://search-ss.wpi.edu/)

## Usage

To integrate this into your FreeBSD kernel:

1. Copy the files to `/usr/src/sys/netinet/cc/`.
2. Rebuild the kernel or kernel module.
3. Load the new congestion control using `kldload` or compile directly into the kernel.
4. Set it using:
   ```bash
   sysctl net.inet.tcp.cc.algorithm=newreno_search

