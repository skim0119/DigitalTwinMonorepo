# Spirob GPIO stream

On the Raspberry Pi 4 Model B, build with a C++17 compiler, make, and
Boost.Beast headers. The process needs read and write access to `/dev/gpiomem`.

```sh
make
./stream-gpio
./stream-gpio -p 5051
```

The server listens on IPv4 loopback (port 5050 by default). To leave it
running after SSH logout, start it from this directory.

```sh
nohup ./stream-gpio >stream.log 2>&1 </dev/null &
echo "$!" >stream.pid
```

Stop that process with `kill "$(cat stream.pid)"` while it is still running.
This launch method does not restart the service after a Pi reboot.

Each WebSocket text frame contains a snapshot, for example
`{"unix_ms":1730000000000,"levels":262144}`. Bit N is the sampled level
of BCM GPIO N (0–27). The streamer samples the Pi 4’s GPIO level
register at about 20 Hz.
Short motor and HX711 pulses may fall between snapshots.

The streamer drives all six HX711 clocks, selects channel A at gain 64, and applies
the existing per-motor offsets and counts-per-kg factors in `stream.cpp`
to each five-conversion average. Stop other HX711 readers before starting the
streamer. The dashboard works with the PID stopped.

For force control, run `python3 Spirob_Code/example/PID_multi.py --motor 0`
from the Pi repo root after starting the streamer. The PID
fetches the latest measurements over local UDP, separately from the
WebSocket refresh. It waits up to five seconds for fresh readings on
all selected channels before starting and stops on a failed fetch or a
reading at least one second old. For a custom port, set
`SPIROB_MONITOR_PORT=5051` to match `./stream-gpio -p 5051`.

The WebSocket `hx711` array is indexed by motor (0–5). Each entry contains
`raw` and `kg`, or is `null` when no reading is available or the last one
is over two seconds old.
