# Real-Time Kernel (PREEMPT_RT) for Medical Acquisition

## Why PREEMPT_RT?

Standard Linux uses a `VOLUNTARY` or `PREEMPT` kernel which has scheduler latency
spikes of 1–50 ms. For continuous biosignal acquisition (EEG 250 Hz, ECG 500 Hz),
a 50 ms spike means **12 missed EEG samples** in one burst.

`PREEMPT_RT` converts nearly all kernel code to preemptible, reducing worst-case
latency to **50–200 µs** on ARM — suitable for biosignal acquisition.

`iio_triggered_bridge` uses epoll + kernel DMA buffer so it is already hardware-timed
(the kernel handles the ADC timing). PREEMPT_RT reduces the latency from kernel
interrupt to userspace `epoll_wait()` wakeup, ensuring the ROS 2 publish happens
within microseconds of the hardware sample being ready.

---

## Kernel Configuration

Add to your kernel config (Yocto: `linux-*.bbappend`, or `make menuconfig`):

```
CONFIG_PREEMPT_RT=y          # Full real-time preemption
CONFIG_HZ=1000               # 1 kHz tick (1 ms resolution)
CONFIG_NO_HZ_FULL=n          # Keep tick running (more predictable for RT)
CONFIG_IRQ_FORCED_THREADING=y  # Force all interrupt handlers to threaded
```

For Orange Pi Zero 2W (Allwinner H618):
```
CONFIG_ARCH_SUNXI=y
CONFIG_PREEMPT_RT=y
CONFIG_HZ_1000=y
```

---

## Verify RT Kernel is Running

```bash
uname -a
# Should contain "PREEMPT_RT" in the version string

# Check scheduler latency (requires rt-tests package)
sudo cyclictest -m -sp98 -d0 -i200 -h400 -n --duration=60s
# Target: max latency < 200 µs on Orange Pi H618
```

---

## Thread Priority for iio_triggered_bridge

After launching the node, set the worker thread to RT priority:

```bash
# Find the iio_triggered_bridge PID
PID=$(pgrep -f iio_triggered_bridge)

# Set real-time scheduler SCHED_FIFO priority 80
sudo chrt -f -p 80 $PID
```

Or launch with elevated priority:

```bash
sudo chrt -f 80 ros2 run ros2_iio_medical iio_triggered_bridge \
  --ros-args -p sysfs_path:=/sys/bus/iio/devices/iio:device0
```

---

## Yocto Integration

Add to your BSP layer:

```bash
# In linux-<machine>.bbappend
KERNEL_FEATURES:append = " features/preempt-rt/preempt-rt.scc"
```

Or use `meta-realtime` layer:
```bash
# In bblayers.conf
BBLAYERS += " /path/to/meta-realtime"
```

---

## Hardware Watchdog

`iio_triggered_bridge` supports Linux hardware watchdog (`/dev/watchdog`) to
automatically reset the system if the acquisition loop hangs.

Enable at launch:

```bash
ros2 run ros2_iio_medical iio_triggered_bridge \
  --ros-args \
  -p sysfs_path:=/sys/bus/iio/devices/iio:device0 \
  -p watchdog_device:=/dev/watchdog \
  -p watchdog_timeout_sec:=5
```

The worker thread kicks the watchdog every `watchdog_timeout_sec / 2` seconds.
If the thread deadlocks or the process crashes, the hardware watchdog resets the board.

**IEC 62304 relevance:** Hardware watchdog is a risk control measure for
FMEA failure mode FM-07 (software hang during acquisition).

---

## PREEMPT_RT + Watchdog Together (Recommended for Production)

```
PREEMPT_RT kernel
    → deterministic wakeup latency (< 200 µs)
    → iio_triggered_bridge worker at SCHED_FIFO priority 80
    → hardware watchdog with 5 s timeout
    → if thread hangs → watchdog fires → board resets → systemd restarts node
```

This combination meets the reliability requirements for Class B medical device
software (IEC 62304) continuous monitoring applications.
