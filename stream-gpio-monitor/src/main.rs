mod web;

use serde::{Deserialize, Serialize};
use std::{
    collections::VecDeque,
    error::Error,
    io::{self, ErrorKind, Read},
    net::{SocketAddr, TcpListener, TcpStream},
    process::{Child, Command, Stdio},
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant},
};
use tungstenite::{Message, client};

const TIMEOUT: Duration = Duration::from_secs(3);

#[derive(Deserialize)]
struct Sample {
    unix_ms: u64,
    levels: u32,
}

#[derive(Clone, Copy)]
struct Channel {
    alpha: u8,
    average: Option<f64>,
}

impl Channel {
    fn update(&mut self, high: bool) {
        let value = f64::from(u8::from(high));
        let alpha = f64::from(self.alpha) / 100.0;
        self.average = Some(
            self.average
                .map_or(value, |old| old + alpha * (value - old)),
        );
    }
}

#[derive(Clone, Serialize)]
struct Point {
    id: u64,
    t: u64,
    levels: u32,
    ema: [u8; 28],
}

struct State {
    channels: [Channel; 28],
    sample: Option<Sample>,
    rtt: Option<Duration>,
    status: String,
    history: VecDeque<Point>,
    next_id: u64,
}

impl Default for State {
    fn default() -> Self {
        Self {
            channels: [Channel {
                alpha: 20,
                average: None,
            }; 28],
            sample: None,
            rtt: None,
            status: "Connecting".into(),
            history: VecDeque::new(),
            next_id: 1,
        }
    }
}

impl State {
    fn update(&mut self, sample: Sample) -> bool {
        if self
            .sample
            .as_ref()
            .is_some_and(|old| old.unix_ms == sample.unix_ms)
        {
            return false;
        }
        for (pin, channel) in self.channels.iter_mut().enumerate() {
            channel.update(sample.levels & (1 << pin) != 0);
        }
        let t = sample.unix_ms;
        if self.history.back().is_some_and(|last| t < last.t) {
            self.history.clear();
        }
        while self.history.len() >= 400
            || self
                .history
                .front()
                .is_some_and(|first| t - first.t > 10_000)
        {
            self.history.pop_front();
        }
        self.history.push_back(Point {
            id: self.next_id,
            t,
            levels: sample.levels,
            ema: self
                .channels
                .map(|channel| (channel.average.unwrap_or_default() * 100.0).round() as u8),
        });
        self.next_id += 1;
        self.sample = Some(sample);
        self.status = "Live".into();
        true
    }

    fn disconnected(&mut self, error: &dyn Error) {
        self.status = format!("Reconnecting ({error})");
        self.sample = None;
        self.rtt = None;
        for channel in &mut self.channels {
            channel.average = None;
        }
    }
}

fn receive(address: SocketAddr, shared: &Mutex<State>) -> Result<(), Box<dyn Error>> {
    let tcp = TcpStream::connect_timeout(&address, TIMEOUT)?;
    tcp.set_read_timeout(Some(TIMEOUT))?;
    tcp.set_write_timeout(Some(TIMEOUT))?;
    let (mut socket, _) = client(format!("ws://{address}/"), tcp)?;
    socket
        .get_ref()
        .set_read_timeout(Some(Duration::from_millis(200)))?;

    let mut last_sample = Instant::now();
    let mut next_ping = Instant::now();
    let mut pending_ping = None;
    let mut nonce = 0_u64;
    loop {
        let now = Instant::now();
        if now.duration_since(last_sample) >= TIMEOUT {
            return Err(io::Error::new(ErrorKind::TimedOut, "no fresh GPIO samples").into());
        }
        if now >= next_ping {
            nonce = nonce.wrapping_add(1);
            socket.send(Message::Ping(nonce.to_be_bytes().to_vec().into()))?;
            if pending_ping.replace((nonce.to_be_bytes(), now)).is_some() {
                shared.lock().unwrap().rtt = None;
            }
            next_ping = now + Duration::from_secs(1);
        }

        match socket.read() {
            Ok(Message::Text(text)) => {
                let sample: Sample = serde_json::from_str(text.as_str())?;
                if shared.lock().unwrap().update(sample) {
                    last_sample = Instant::now();
                }
            }
            Ok(Message::Pong(payload)) => {
                if let Some((expected, sent)) = pending_ping
                    && payload.as_ref() == expected
                {
                    shared.lock().unwrap().rtt = Some(sent.elapsed());
                    pending_ping = None;
                }
            }
            Ok(Message::Close(_)) => return Err("stream closed".into()),
            Ok(_) => (),
            Err(tungstenite::Error::Io(error))
                if matches!(error.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
            Err(error) => return Err(error.into()),
        }
    }
}

struct Tunnel {
    child: Child,
    host: String,
    forward: String,
}

impl Tunnel {
    fn connect(host: &str, port: u16) -> Result<(SocketAddr, Self), Box<dyn Error>> {
        let listener = TcpListener::bind(("127.0.0.1", 0))?;
        let address = listener.local_addr()?;
        drop(listener);

        let forward = format!("{address}:127.0.0.1:{port}");
        let child = Command::new("ssh")
            .args([
                "-T",
                "-o",
                "BatchMode=yes",
                "-o",
                "ExitOnForwardFailure=yes",
                "-L",
                &forward,
                "--",
                host,
                "cat",
            ])
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::piped())
            .spawn()?;
        let mut tunnel = Self {
            child,
            host: host.into(),
            forward,
        };
        // The open stdin keeps the SSH session alive even on a multiplexed connection.
        let deadline = Instant::now() + TIMEOUT;
        loop {
            if tunnel.child.try_wait()?.is_some() {
                let mut error = String::new();
                tunnel
                    .child
                    .stderr
                    .take()
                    .unwrap()
                    .read_to_string(&mut error)?;
                return Err(format!("SSH tunnel to {host} exited: {}", error.trim()).into());
            }
            if TcpStream::connect(address).is_ok() {
                return Ok((address, tunnel));
            }
            if Instant::now() >= deadline {
                return Err(format!("Timed out opening SSH tunnel to {host}").into());
            }
            thread::sleep(Duration::from_millis(50));
        }
    }
}

impl Drop for Tunnel {
    fn drop(&mut self) {
        // A multiplexed forward belongs to the SSH master and needs explicit cancellation.
        let _ = Command::new("ssh")
            .args(["-O", "cancel", "-L", &self.forward, "--", &self.host])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let target = args.next().unwrap_or_else(|| "spirob:5050".into());
    if target == "--help" {
        println!("Usage: stream-gpio-monitor [SSH_HOST:PORT | IP:PORT]");
        return Ok(());
    }
    if args.next().is_some() {
        return Err("Usage: stream-gpio-monitor [SSH_HOST:PORT | IP:PORT]".into());
    }
    let (address, _tunnel) = if let Ok(address) = target.parse::<SocketAddr>() {
        (address, None)
    } else {
        let (host, port) = target.rsplit_once(':').ok_or("Expected SSH_HOST:PORT")?;
        let port = port.parse::<u16>()?;
        if host.is_empty() || port == 0 {
            return Err("Expected an SSH host and a nonzero port".into());
        }
        let (address, tunnel) = Tunnel::connect(host, port)?;
        (address, Some(tunnel))
    };
    let shared = Arc::new(Mutex::new(State::default()));
    let worker = Arc::clone(&shared);
    thread::spawn(move || {
        loop {
            if let Err(error) = receive(address, &worker) {
                worker.lock().unwrap().disconnected(error.as_ref());
            }
            thread::sleep(Duration::from_secs(1));
        }
    });

    web::serve(target, shared).await?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn smooths_each_pin_once_per_sample() {
        let mut state = State::default();
        state.channels[2].alpha = 50;
        assert!(state.update(Sample {
            unix_ms: 1,
            levels: 0
        }));
        assert!(state.update(Sample {
            unix_ms: 2,
            levels: (1 << 2) | (1 << 3)
        }));
        assert!(!state.update(Sample {
            unix_ms: 2,
            levels: 0
        }));
        assert_eq!(state.channels[2].average, Some(0.5));
        assert_eq!(state.channels[3].average, Some(0.2));
        assert_eq!(state.channels[4].average, Some(0.0));
        assert_eq!(state.history.len(), 2);
        assert_eq!(state.history[1].ema[2], 50);
        state.update(Sample {
            unix_ms: 10_003,
            levels: 0,
        });
        assert_eq!(state.history.len(), 1);
        assert_eq!(state.history[0].id, 3);
    }
}
