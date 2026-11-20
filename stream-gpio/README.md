# Spirob GPIO stream

On the Raspberry Pi 4 Model B, build with a C++17 compiler, make, and
Boost.Beast headers. The process needs read access to `/dev/gpiomem`.

```sh
make
./run-stream
./run-stream -p 5051
```

The server listens on IPv4 loopback (port 5050 by default). To leave it
running after SSH logout, start it from this directory.

```sh
nohup ./run-stream >stream.log 2>&1 </dev/null &
echo "$!" >stream.pid
```

Stop that process with `kill "$(cat stream.pid)"` while it is still running.
This launch method does not restart the service after a Pi reboot.

Each WebSocket text frame contains a snapshot, for example
`{"unix_ms":1730000000000,"levels":262144}`. Bit N is the sampled level
of BCM GPIO N (0–27). The sampler reads the Pi 4's GPIO level register
at about 20 Hz without requesting pins or changing their configuration.
Short motor and HX711 pulses may fall between snapshots.
