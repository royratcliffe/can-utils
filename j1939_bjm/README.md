# J1939 Basic Joystick Message Server

`j1939-bjm-srv` broadcasts joystick positions and button states as J1939
Basic Joystick Messages (BJM1 and BJM2). The accompanying [systemd
service](j1939-bjm-srv.service) configures real-time scheduling and can
be pinned to an isolated CPU to reduce scheduling jitter.

## Install the service

Install the server executable at `/usr/bin/j1939-bjm-srv` first. From
the repository root:

``` sh
sudo install -m 644 j1939_bjm/j1939-bjm-srv.service /etc/systemd/system/
sudo systemctl daemon-reload
```

The service defaults to CAN interface `can0` and local address `0x90`.
Optional overrides belong in `/etc/default/j1939-bjm-srv`:

``` ini
INTERFACE=can0
LOCAL_ADDR=0x90
OPTIONS="--joy1 /dev/input/js0"
```

Configure and bring up the CAN interface separately before starting the
server. Use a local address appropriate for your network. An explicitly
specified joystick must exist and be readable.

If changing the interface, also replace the unit's `BindsTo=` and
`After=` device dependencies in a systemd override; changing `INTERFACE`
alone does not update them.

``` sh
sudo systemctl enable --now j1939-bjm-srv
systemctl status j1939-bjm-srv
journalctl -u j1939-bjm-srv -b
```

## Isolate a CPU on Raspberry Pi

The following example reserves CPU **3** on a four-core Raspberry Pi,
leaving CPUs **0-2** for normal workloads and interrupt handling. CPU
numbers start at zero. Check the available CPUs before adapting it:

``` sh
lscpu -e=CPU,ONLINE
```

CPU isolation affects the whole system and requires a reboot. Perform
these steps during a suitable maintenance window.

### 1. Configure the kernel command line

Recent Raspberry Pi OS installations use `/boot/firmware/cmdline.txt`.
Older installations may use `/boot/cmdline.txt`. Edit the active file,
not a placeholder directing you to another location.

``` sh
sudo cp /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt.bak
sudo nano /boot/firmware/cmdline.txt
```

Append these options, separated by spaces, to the existing line:

``` text
isolcpus=domain,managed_irq,3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2
```

Keep all existing boot options. The entire command line must remain on
**one line**.

| Option | Purpose |
|------------------------------------|------------------------------------|
| `isolcpus=domain,managed_irq,3` | Remove CPU 3 from scheduler load balancing and avoid managed IRQs there where possible. |
| `nohz_full=3` | Reduce periodic scheduler ticks on CPU 3 when kernel conditions permit. |
| `rcu_nocbs=3` | Offload CPU 3's RCU callbacks to kernel threads. |
| `irqaffinity=0-2` | Set the default interrupt affinity to CPUs 0-2. |

Full tick suppression (`nohz_full`) reduces periodic timer interrupts on
an otherwise idle CPU; RCU callback offloading (`rcu_nocbs`) moves RCU
cleanup work to other CPUs. Both require corresponding kernel support.
These options reduce interference; they do not guarantee
an interrupt-free CPU. Managed IRQ avoidance is best-effort, and
drivers, IRQ affinity tools or kernel threads can require further
tuning.

### 2. Pin the service to CPU 3

Create an override for the installed unit:

``` sh
sudo systemctl edit j1939-bjm-srv
```

Add:

``` ini
[Service]
CPUAffinity=3
```

Alternatively, uncomment `CPUAffinity=3` in the [service
file](j1939-bjm-srv.service) before installing it.

Isolation removes automatic load balancing onto CPU 3; it does not
automatically place the server there or prevent other explicitly pinned
tasks from using it. `CPUAffinity=3` supplies the server's placement.

Reload systemd and reboot to apply the kernel options:

``` sh
sudo systemctl daemon-reload
sudo reboot
```

### 3. Verify after reboot

``` sh
cat /proc/cmdline
cat /sys/devices/system/cpu/isolated
cat /sys/devices/system/cpu/nohz_full
systemctl status j1939-bjm-srv
systemctl show j1939-bjm-srv --property=CPUAffinity
```

The isolated CPU list should contain `3`. With full tick suppression
supported and enabled, the `nohz_full` list should also contain `3`. If
that file is absent or the expected CPU is missing, check the kernel
configuration and boot messages rather than assuming the option worked.

With the service running, check its actual process affinity:

``` sh
taskset -pc "$(systemctl show j1939-bjm-srv --property=MainPID --value)"
```

The allowed CPU list should be `3`. To check interrupt placement, compare
per-CPU counts in `/proc/interrupts` before and during a representative
test. Counts increasing in CPU 3's column—especially for the CAN
controller or the joystick's input path, often USB—show that interrupts
are still reaching the isolated CPU. IRQ names vary by hardware; also
check threaded IRQs and any configured affinity. Measure joystick
event-to-CAN transmission latency under representative system load, and
look for high or irregular delays and outliers, not just the average.

## Real-time considerations

CPU isolation does not install or enable a PREEMPT_RT kernel. The unit
requests `SCHED_FIFO` priority 70, but this alone does not guarantee
bounded latency.

On PREEMPT_RT, many interrupt handlers run as schedulable threads.
Ensure the joystick and CAN interrupt threads can run promptly: a FIFO
process can starve lower-priority work on the same CPU. Do not assume
priority 70 is suitable for every IRQ placement and system workload.

Keep unrelated services off CPU 3 and avoid pinning multiple busy
real-time processes there. Tune IRQ placement and priorities based on
measurements, not just the presence of the isolation settings.

## Undo CPU isolation

Remove the added `isolcpus`, `nohz_full`, `rcu_nocbs` and `irqaffinity`
options from the active boot command line, preserving its other options.
Remove `CPUAffinity=3` from the service override, or use an empty
`CPUAffinity=` assignment in an override if it was set in the main unit.

Run `sudo systemctl daemon-reload` and reboot. Keep any unrelated
service overrides intact.
