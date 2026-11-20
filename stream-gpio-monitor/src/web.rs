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
    collections::HashMap,
    convert::Infallible,
    io,
    path::Path,
    process::{Command, Stdio},
    sync::{Arc, Mutex},
    time::Duration,
};

const HOST: &str = "127.0.0.1:8080";
const DASHBOARD_PAGE: &str = r#"<!doctype html>
<html lang="en">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Spirob GPIO monitor</title>
<link rel="stylesheet" href="/dashboard.css">
<div id="main"></div>
<script type="module">
  import("/dashboard-assets/dashboard.js").then(({ default: init }) => init()).catch(() => {
    document.getElementById("main").textContent = "Could not load the dashboard.";
  });
</script>
</html>"#;

type DashboardAssets = HashMap<String, (&'static str, Bytes)>;

fn load_dashboard_assets(
    dist: &Path,
    directory: &Path,
    assets: &mut DashboardAssets,
) -> io::Result<()> {
    for entry in std::fs::read_dir(directory)? {
        let entry = entry?;
        let file_type = entry.file_type()?;
        let path = entry.path();
        if file_type.is_dir() {
            load_dashboard_assets(dist, &path, assets)?;
        } else if file_type.is_file() {
            let content_type = match path.extension().and_then(|extension| extension.to_str()) {
                Some("js") => "text/javascript; charset=utf-8",
                Some("wasm") => "application/wasm",
                _ => continue,
            };
            let name = path.strip_prefix(dist).unwrap().to_string_lossy();
            assets.insert(
                format!("/dashboard-assets/{name}"),
                (content_type, std::fs::read(&path)?.into()),
            );
        }
    }
    Ok(())
}

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
    assets: &DashboardAssets,
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
        (&Method::GET, "/") => response(StatusCode::OK, "text/html; charset=utf-8", DASHBOARD_PAGE),
        (&Method::GET, "/dashboard.css") => response(
            StatusCode::OK,
            "text/css; charset=utf-8",
            include_str!("../dashboard/src/dashboard.css"),
        ),
        (&Method::GET, path) if path.starts_with("/dashboard-assets/") => match assets.get(path) {
            Some((content_type, bytes)) => response(StatusCode::OK, content_type, bytes.clone()),
            None => response(StatusCode::NOT_FOUND, text, "Not found."),
        },
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
                "hx711": state.sample.as_ref().map(|sample| &sample.hx711),
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
    let mut assets = DashboardAssets::new();
    let dist = Path::new(env!("CARGO_MANIFEST_DIR")).join("dashboard/dist");
    load_dashboard_assets(&dist, &dist, &mut assets).map_err(|error| {
        io::Error::new(
            error.kind(),
            format!("Could not load dashboard assets. Run make dashboard: {error}"),
        )
    })?;
    if !assets.contains_key("/dashboard-assets/dashboard.js")
        || !assets.contains_key("/dashboard-assets/dashboard_bg.wasm")
    {
        return Err(io::Error::new(
            io::ErrorKind::NotFound,
            "Dashboard assets are missing. Run make dashboard.",
        ));
    }
    let assets = Arc::new(assets);
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
                let assets = Arc::clone(&assets);
                tokio::spawn(async move {
                    let _ = http1::Builder::new()
                        .timer(TokioTimer::new())
                        .header_read_timeout(Duration::from_secs(5))
                        .serve_connection(TokioIo::new(stream), service_fn(move |request| {
                            std::future::ready(Ok::<_, Infallible>(handle(request, &target, &shared, &assets)))
                        })).await;
                });
            }
        }
    }
}
