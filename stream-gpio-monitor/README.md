# Spirob GPIO monitor

Install the WebAssembly target and the matching binding generator once:

```sh
rustup target add wasm32-unknown-unknown
cargo install wasm-bindgen-cli --version 0.2.130 --locked
```

From this directory, build the Dioxus dashboard and start the monitor:

```sh
make dashboard
cargo run
```

The Rust monitor opens the dashboard at [127.0.0.1:8080](http://127.0.0.1:8080)
and connects to `spirob:5050` over SSH automatically. SSH must already work
without prompts. Press Ctrl+C in the terminal to stop the monitor.

After editing the dashboard, rerun `make dashboard` and restart the monitor.
If Homebrew's Rust comes first on your PATH, run `RUSTC="$(rustup which rustc)" make dashboard`
to build with the toolchain that has the WebAssembly target.

Use `cargo run -- spirob:5051` to change the remote port. A numeric address
such as `127.0.0.1:5051` connects directly, for example to a manual tunnel.

- **HX711** displays raw counts and calibrated load in kg for each motor
  when the Pi GPIO streamer runs with `--hx711`.
  It plots the last 10 seconds of load in kg alongside an EMA that defaults
  to alpha 0.20. Each load graph has a scroll-adjustable EMA slider.
  Each motor has its own vertical scale. Missing readings leave gaps.
  **Noise (g)** is the RMS difference between measured load and a fixed 0.20 EMA over
  the same window, in grams. Rapid changes in load also raise this value.
  The tab also displays DOUT and SCK pin levels.
- **Motor outputs** displays PUL and DIR pin levels.
- Scroll over a signal's slider or use the slider's arrow keys to adjust
  EMA alpha from 0.01 to 1.00.

The load sliders redraw the visible EMA. The GPIO sliders change smoothing
for incoming samples. GPIO graphs show the last 10 seconds of sampled
levels and the EMA. RTT is the stream's WebSocket ping/pong round-trip time.
EMA high is a smoothed share of sampled HIGH levels, with 0.20 as the default alpha.
Fast pulses can be missed between samples.
