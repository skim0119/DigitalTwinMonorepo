use crate::State;
use http_body_util::Full;
use hyper::{
    Method, Request, Response, StatusCode,
    body::{Bytes, Incoming},
    server::conn::http1,
    service::service_fn,
};
use hyper_util::rt::{TokioIo, TokioTimer};
use serde_json::json;
use std::{
    convert::Infallible,
    io,
    process::{Command, Stdio},
    sync::{Arc, Mutex},
    time::Duration,
};

const HOST: &str = "127.0.0.1:8080";

fn response(
    status: StatusCode,
    content_type: &str,
    body: impl Into<Bytes>,
) -> Response<Full<Bytes>> {
    Response::builder()
        .status(status)
        .header("content-type", content_type)
        .header("cache-control", "no-store")
        .header("x-content-type-options", "nosniff")
        .body(Full::new(body.into()))
        .unwrap()
}

fn handle(
    request: Request<Incoming>,
    target: &str,
    shared: &Mutex<State>,
) -> Response<Full<Bytes>> {
    let text = "text/plain; charset=utf-8";
    if request
        .headers()
        .get("host")
        .is_none_or(|host| host != HOST)
    {
        return response(
            StatusCode::FORBIDDEN,
            text,
            "Use the printed dashboard URL.",
        );
    }

    match (request.method(), request.uri().path()) {
        (&Method::GET, "/") => response(
            StatusCode::OK,
            "text/html; charset=utf-8",
            include_str!("dashboard.html"),
        ),
        (&Method::GET, "/api/state") => {
            let after = request
                .uri()
                .query()
                .and_then(|query| query.strip_prefix("after="))
                .and_then(|value| value.parse::<u64>().ok())
                .unwrap_or_default();
            let state = shared.lock().unwrap();
            let newest = state.next_id - 1;
            let reset = after > newest;
            let history: Vec<_> = state
                .history
                .iter()
                .filter(|point| reset || point.id > after)
                .collect();
            let levels = state.sample.as_ref().map(|sample| sample.levels);
            let channels: Vec<_> = state
                .channels
                .iter()
                .enumerate()
                .map(|(pin, channel)| {
                    json!({
                        "level": levels.map(|bits| bits & (1 << pin) != 0),
                        "ema": channel.average,
                        "alpha": channel.alpha,
                    })
                })
                .collect();
            let payload = json!({
                "target": target,
                "status": state.status,
                "live": levels.is_some(),
                "rtt_ms": state.rtt.map(|rtt| rtt.as_secs_f64() * 1000.0),
                "channels": channels,
                "history": history,
                "reset": reset,
            });
            response(StatusCode::OK, "application/json", payload.to_string())
        }
        (&Method::PUT, path) if path.starts_with("/api/alpha/") => {
            if request
                .headers()
                .get("x-gpio-monitor")
                .is_none_or(|value| value != "1")
            {
                return response(StatusCode::FORBIDDEN, text, "Missing dashboard header.");
            }
            if let Some((pin, alpha)) = path["/api/alpha/".len()..].split_once('/')
                && let (Ok(pin), Ok(alpha)) = (pin.parse::<usize>(), alpha.parse::<u8>())
                && (1..=100).contains(&alpha)
                && let Some(channel) = shared.lock().unwrap().channels.get_mut(pin)
            {
                channel.alpha = alpha;
                return response(StatusCode::NO_CONTENT, text, "");
            }
            response(StatusCode::BAD_REQUEST, text, "Invalid GPIO or alpha.")
        }
        _ => response(StatusCode::NOT_FOUND, text, "Not found."),
    }
}

pub async fn serve(target: String, shared: Arc<Mutex<State>>) -> io::Result<()> {
    let listener = tokio::net::TcpListener::bind(HOST).await?;
    let url = format!("http://{HOST}");
    println!("Spirob GPIO dashboard: {url}  (Ctrl+C to stop)");
    let opener = if cfg!(target_os = "macos") {
        "open"
    } else {
        "xdg-open"
    };
    if let Ok(mut child) = Command::new(opener)
        .arg(&url)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
    {
        std::thread::spawn(move || {
            let _ = child.wait();
        });
    }

    let target: Arc<str> = target.into();
    let shutdown = tokio::signal::ctrl_c();
    tokio::pin!(shutdown);
    let mut terminate = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
    loop {
        tokio::select! {
            result = &mut shutdown => return result,
            _ = terminate.recv() => return Ok(()),
            incoming = listener.accept() => {
                let (stream, _) = incoming?;
                let (target, shared) = (Arc::clone(&target), Arc::clone(&shared));
                tokio::spawn(async move {
                    let _ = http1::Builder::new()
                        .timer(TokioTimer::new())
                        .header_read_timeout(Duration::from_secs(5))
                        .serve_connection(TokioIo::new(stream), service_fn(move |request| {
                            std::future::ready(Ok::<_, Infallible>(handle(request, &target, &shared)))
                        })).await;
                });
            }
        }
    }
}
