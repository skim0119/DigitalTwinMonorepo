use serde::{Deserialize, Serialize};
use std::{
    error::Error,
    io::ErrorKind,
    net::{TcpListener, TcpStream},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
use tungstenite::{Message, accept};

const ADDRESS: &str = "127.0.0.1:5051";
const PERIOD: Duration = Duration::from_millis(50);
const OFFSETS: [f64; 6] = [170539.0, -285116.4, 276320.0, 10569.0, 236592.0, -170491.0];
const COUNTS_PER_KG: [f64; 6] = [34170.0, 34170.0, 39293.8, 34824.0, 33760.0, 31950.0];

#[derive(Deserialize)]
struct Pins {
    motor: usize,
    pul: u32,
    dir: u32,
    dout: u32,
    sck: u32,
}

#[derive(Deserialize)]
struct Wiring {
    motors: [Pins; 6],
}

impl Wiring {
    fn read() -> Result<Self, Box<dyn Error>> {
        let wiring: Self = serde_json::from_str(include_str!("../../pins.json"))?;
        let mut used = 0;
        for (index, pins) in wiring.motors.iter().enumerate() {
            if pins.motor != index {
                return Err("Expected pins.json motor indices in order from 0 to 5".into());
            }
            for pin in [pins.pul, pins.dir, pins.dout, pins.sck] {
                if pin >= 28 || used & (1 << pin) != 0 {
                    return Err(format!("Invalid or duplicate BCM GPIO {pin} in pins.json").into());
                }
                used |= 1 << pin;
            }
        }
        Ok(wiring)
    }
}

#[derive(Serialize)]
struct Load {
    raw: f64,
    kg: f64,
}

#[derive(Serialize)]
struct Sample {
    unix_ms: u64,
    levels: u32,
    hx711: [Load; 6],
}

fn sample(wiring: &Wiring, unix_ms: u64, tick: u64) -> Sample {
    let mut levels = 0;
    let hx711 = std::array::from_fn(|motor| {
        let pins = &wiring.motors[motor];
        let step = tick + motor as u64 * 3;
        let active = step % 100 < 65;
        let reverse = step % 200 >= 100;
        let clock_high = step % 13 == 0;
        levels |= u32::from(active && step % (3 + motor as u64) == 0) << pins.pul;
        levels |= u32::from(reverse) << pins.dir;
        levels |= u32::from(step % 5 < 3) << pins.dout;
        levels |= u32::from(clock_high) << pins.sck;

        // Hold five-frame blocks to mimic the HX711 reader's conversion averaging.
        let time = (tick / 5) as f64 * 0.25;
        let phase = time * (0.7 + motor as f64 * 0.1) + motor as f64;
        let kg = 0.02
            + motor as f64 * 0.025
            + (0.012 + 0.002 * motor as f64) * phase.sin()
            + 0.003 * (phase * 9.0).sin();
        Load {
            kg,
            raw: OFFSETS[motor] + COUNTS_PER_KG[motor] * kg,
        }
    });
    Sample {
        unix_ms,
        levels,
        hx711,
    }
}

fn serve(stream: TcpStream, wiring: &Wiring, started: Instant) -> Result<(), Box<dyn Error>> {
    stream.set_read_timeout(Some(Duration::from_secs(2)))?;
    stream.set_write_timeout(Some(Duration::from_secs(2)))?;
    let mut socket = accept(stream)?;
    let mut next = Instant::now();
    loop {
        let now = Instant::now();
        if now >= next {
            let unix_ms = SystemTime::now().duration_since(UNIX_EPOCH)?.as_millis() as u64;
            let tick = started.elapsed().as_millis() as u64 / PERIOD.as_millis() as u64;
            socket.send(Message::Text(
                serde_json::to_string(&sample(wiring, unix_ms, tick))?.into(),
            ))?;
            next = now + PERIOD;
        }
        socket.get_ref().set_read_timeout(Some(
            next.saturating_duration_since(Instant::now())
                .max(Duration::from_millis(1)),
        ))?;
        match socket.read() {
            Ok(Message::Ping(_)) => socket.flush()?,
            Ok(Message::Close(_)) => return Ok(()),
            Ok(_) => {}
            Err(tungstenite::Error::Io(error))
                if matches!(error.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
            Err(tungstenite::Error::ConnectionClosed | tungstenite::Error::AlreadyClosed) => {
                return Ok(());
            }
            Err(error) => return Err(error.into()),
        }
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let listener = TcpListener::bind(ADDRESS)?;
    let wiring = Wiring::read()?;
    let started = Instant::now();
    println!("Simulated Spirob stream at ws://{ADDRESS}/ (Ctrl+C to stop)");
    std::thread::scope(|scope| {
        for stream in listener.incoming() {
            let stream = stream?;
            let wiring = &wiring;
            scope.spawn(move || {
                if let Err(error) = serve(stream, wiring, started) {
                    eprintln!("Simulator client disconnected: {error}");
                }
            });
        }
        Ok(())
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frames_use_configured_pins_and_change_over_time() {
        let wiring = Wiring::read().unwrap();
        let first = sample(&wiring, 1000, 0);
        let later = sample(&wiring, 6000, 100);
        let pins = &wiring.motors[0];
        assert_eq!(first.levels & (1 << pins.pul), 1 << pins.pul);
        assert_eq!(first.levels & (1 << pins.sck), 1 << pins.sck);
        assert_eq!(first.levels & (1 << pins.dir), 0);
        assert_eq!(later.levels & (1 << pins.dir), 1 << pins.dir);
        assert!(
            first
                .hx711
                .iter()
                .zip(later.hx711)
                .all(|(a, b)| a.kg != b.kg)
        );
    }
}
