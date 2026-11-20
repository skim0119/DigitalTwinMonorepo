# Spirob GPIO monitor

From this directory, run:

```sh
cargo run
```

The Rust monitor opens the dashboard at [127.0.0.1:8080](http://127.0.0.1:8080)
and connects to `spirob:5050` over SSH automatically. SSH must already work
without prompts. Press Ctrl+C in the terminal to stop the monitor.

Use `cargo run -- spirob:5051` to change the remote port. A numeric address
such as `127.0.0.1:5051` connects directly, for example to a manual tunnel.

- **HX711** displays raw counts and calibrated load in kg for each motor
  when the Pi GPIO streamer runs with `--hx711`.
  It plots the last 10 seconds of load in kg alongside an EMA with alpha 0.20.
  Each motor has its own vertical scale. Missing readings leave gaps.
  The tab also displays DOUT and SCK pin levels.
- **Motor outputs** displays PUL and DIR pin levels.
- Scroll over a signal's slider or use the slider's arrow keys to adjust
  EMA alpha from 0.01 to 1.00.

Each graph shows the last 10 seconds of sampled GPIO levels and the EMA.
Changing alpha affects new samples as they arrive. RTT is the stream's WebSocket ping/pong round-trip time. EMA high is a
smoothed share of sampled HIGH levels, with 0.20 as the default alpha.
Fast pulses can be missed between samples.
