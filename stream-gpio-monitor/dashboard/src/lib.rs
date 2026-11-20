use dioxus::prelude::*;
use gloo_net::http::Request;
use gloo_timers::{callback::Timeout, future::TimeoutFuture};
use serde::Deserialize;
use wasm_bindgen::JsCast;
use web_sys::{AbortController, HtmlElement};

mod plot;

const WINDOW_MS: u64 = 10_000;
const GAP_MS: u64 = 250;
const DEFAULT_ALPHA: u8 = 20;
const TABS: [(&str, &str); 2] = [("hx711", "HX711"), ("motor", "Motor outputs")];
const GPIO_PINS: [[[usize; 2]; 6]; 2] = [
    [[2, 3], [5, 6], [7, 11], [21, 13], [17, 20], [12, 19]],
    [[18, 14], [10, 9], [4, 27], [23, 1], [22, 25], [26, 0]],
];
const GPIO_SIGNALS: [[&str; 2]; 2] = [["DOUT", "SCK"], ["PUL", "DIR"]];

#[derive(Deserialize)]
struct Point {
    id: u64,
    t: u64,
    levels: u32,
    ema: [u8; 28],
    kg: [Option<f64>; 6],
    kg_ema: [Option<f64>; 6],
}

#[derive(Deserialize)]
struct Load {
    raw: f64,
    kg: f64,
}

#[derive(Deserialize)]
struct Channel {
    level: Option<bool>,
    ema: Option<f64>,
    alpha: u8,
}

#[derive(Deserialize)]
struct Snapshot {
    target: String,
    status: String,
    live: bool,
    rtt_ms: Option<f64>,
    channels: Vec<Channel>,
    hx711: Option<[Option<Load>; 6]>,
    history: Vec<Point>,
    reset: bool,
}

#[derive(Default)]
struct Dashboard {
    snapshot: Option<Snapshot>,
    history: Vec<Point>,
    disconnected: bool,
}

impl Dashboard {
    fn update(&mut self, mut snapshot: Snapshot) {
        if snapshot.reset
            || snapshot
                .history
                .first()
                .zip(self.history.last())
                .is_some_and(|(next, last)| next.t < last.t)
        {
            self.history.clear();
        }
        self.history.append(&mut snapshot.history);
        if let Some(last) = self.history.last() {
            let end = last.t;
            self.history.retain(|point| end - point.t <= WINDOW_MS);
        }
        self.snapshot = Some(snapshot);
        self.disconnected = false;
    }
}

#[wasm_bindgen::prelude::wasm_bindgen(start)]
pub fn start() {
    dioxus::launch(App);
}

async fn fetch_snapshot(cursor: u64) -> Option<Snapshot> {
    let controller = AbortController::new().ok()?;
    let abort = controller.clone();
    let _timeout = Timeout::new(1500, move || abort.abort());
    let response = Request::get(&format!("/api/state?after={cursor}"))
        .abort_signal(Some(&controller.signal()))
        .send()
        .await
        .ok()?;
    if !response.ok() {
        return None;
    }
    response.json().await.ok()
}

#[component]
fn App() -> Element {
    let mut dashboard = use_context_provider(|| Signal::new(Dashboard::default()));
    let error = use_context_provider(|| Signal::new(String::new()));
    let mut tab = use_signal(|| 0);
    use_future(move || async move {
        loop {
            let cursor = dashboard.peek().history.last().map_or(0, |point| point.id);
            match fetch_snapshot(cursor).await {
                Some(snapshot) => dashboard.write().update(snapshot),
                None => {
                    let mut state = dashboard.write();
                    state.snapshot = None;
                    state.disconnected = true;
                }
            }
            TimeoutFuture::new(100).await;
        }
    });

    let state = dashboard.read();
    let snapshot = state.snapshot.as_ref();
    let status = snapshot.map_or_else(
        || {
            if state.disconnected {
                "Dashboard disconnected".into()
            } else {
                "Connecting…".into()
            }
        },
        |snapshot| format!("{} · {}", snapshot.target, snapshot.status),
    );
    let live = snapshot.is_some_and(|snapshot| snapshot.live);
    let rtt = snapshot
        .and_then(|snapshot| snapshot.rtt_ms)
        .map_or_else(|| "—".into(), |ms| format!("{ms:.1} ms"));
    let active = tab();
    rsx! {
        main {
            header {
                div {
                    h1 { "Spirob GPIO monitor" }
                    p { id: "status", role: "status", class: if live { "live" }, "{status}" }
                }
                p { class: "metric", title: "WebSocket ping/pong round-trip time",
                    span { "WebSocket RTT" }
                    strong { id: "rtt", "{rtt}" }
                }
            }
            nav { role: "tablist", aria_label: "GPIO groups",
                for (index, (id, label)) in TABS.into_iter().enumerate() {
                    button {
                        id: "{id}-tab", role: "tab", aria_controls: "{id}",
                        aria_selected: if active == index { "true" } else { "false" },
                        tabindex: if active == index { "0" } else { "-1" },
                        onclick: move |_| tab.set(index),
                        onkeydown: move |event| {
                            let next = match event.key() {
                                Key::Home => 0,
                                Key::End => 1,
                                Key::ArrowLeft | Key::ArrowRight => 1 - index,
                                _ => return,
                            };
                            event.prevent_default();
                            tab.set(next);
                            if let Some(button) = web_sys::window().and_then(|w| w.document())
                                .and_then(|d| d.get_element_by_id(&format!("{}-tab", TABS[next].0)))
                                .and_then(|el| el.dyn_into::<HtmlElement>().ok())
                            {
                                let _ = button.focus();
                            }
                        },
                        "{label}"
                    }
                }
            }
            div { class: "plot-key",
                span { i { aria_hidden: "true" } "Measured load / GPIO level" }
                span { i { class: "smooth", aria_hidden: "true" } "EMA" }
            }
            section { id: "hx711", role: "tabpanel", aria_labelledby: "hx711-tab", hidden: active != 0,
                Loads { visible: active == 0 }
                Gpio { group: 0, visible: active == 0 }
            }
            section { id: "motor", role: "tabpanel", aria_labelledby: "motor-tab", hidden: active != 1,
                Gpio { group: 1, visible: active == 1 }
            }
            footer {
                p { "EMA high is the smoothed share of HIGH samples. Scroll over an alpha slider or use its arrow keys to adjust smoothing. Short pulses can fall between samples." }
                p { id: "error", role: "alert", "{error}" }
            }
        }
    }
}

#[component]
fn TimeAxis() -> Element {
    rsx! { div { class: "axis", span { "−10 s" } span { "−5 s" } span { "Latest" } } }
}

#[component]
fn Loads(visible: bool) -> Element {
    rsx! {
        div { class: "table-wrap", tabindex: "0", aria_label: "HX711 loads",
            table {
                thead { tr {
                    th { "Motor" }
                    th { "Raw counts" }
                    th { "Load (kg)" }
                    th { title: "10-second RMS difference from the reference EMA (alpha 0.20)", "Noise (g)" }
                    th { class: "plot-column", "Load history (kg)" TimeAxis {} }
                    th { "EMA alpha" }
                } }
                tbody {
                    for motor in 0..6 {
                        LoadRow { key: "{motor}", motor, visible }
                    }
                }
            }
        }
    }
}

#[component]
fn LoadRow(motor: usize, visible: bool) -> Element {
    let mut alpha = use_signal(|| DEFAULT_ALPHA);
    let dashboard = use_context::<Signal<Dashboard>>();
    let state = dashboard.read();
    let load = state
        .snapshot
        .as_ref()
        .and_then(|snapshot| snapshot.hx711.as_ref())
        .and_then(|loads| loads[motor].as_ref());
    let raw = load.map_or_else(|| "—".into(), |load| format!("{:.1}", load.raw));
    let kg = load.map_or_else(|| "—".into(), |load| format!("{:.3}", load.kg));
    let noise = load
        .and_then(|_| plot::noise_grams(&state.history, motor))
        .map_or_else(|| "—".into(), |noise| format!("{noise:.1}"));
    let selected_alpha = alpha();
    rsx! {
        tr {
            th { scope: "row", "Motor {motor}" }
            td { "{raw}" }
            td { "{kg}" }
            td { "{noise}" }
            td {
                if visible {
                    {plot::load_plot(&state.history, motor, selected_alpha)}
                }
            }
            td {
                AlphaSlider {
                    label: format!("Motor {motor} load EMA alpha"), value: selected_alpha,
                    onchange: move |value| alpha.set(value),
                }
            }
        }
    }
}

#[component]
fn Gpio(group: usize, visible: bool) -> Element {
    rsx! {
        div { class: "table-wrap", tabindex: "0", aria_label: "GPIO channels",
            table {
                thead { tr {
                    th { "Motor" }
                    th { "Signal" }
                    th { "BCM" }
                    th { class: "plot-column", "GPIO history" TimeAxis {} }
                    th { "Latest" }
                    th { "EMA alpha" }
                } }
                for (motor, pins) in GPIO_PINS[group].into_iter().enumerate() {
                    tbody { key: "{motor}",
                        for (line, pin) in pins.into_iter().enumerate() {
                            GpioRow { key: "{pin}", motor, pin, signal: GPIO_SIGNALS[group][line], visible }
                        }
                    }
                }
            }
        }
    }
}

#[component]
fn GpioRow(motor: usize, pin: usize, signal: &'static str, visible: bool) -> Element {
    let mut dashboard = use_context::<Signal<Dashboard>>();
    let mut error = use_context::<Signal<String>>();
    let mut pending = use_signal(|| None::<u8>);
    let state = dashboard.read();
    let channel = state
        .snapshot
        .as_ref()
        .and_then(|snapshot| snapshot.channels.get(pin));
    let alpha = pending()
        .or_else(|| channel.map(|channel| channel.alpha))
        .unwrap_or(DEFAULT_ALPHA);
    let level = channel.and_then(|channel| channel.level);
    let level_text = match level {
        Some(true) => "HIGH",
        Some(false) => "LOW",
        None => "—",
    };
    let ema = channel
        .and_then(|channel| channel.ema)
        .map_or_else(|| "—".into(), |ema| format!("{:.1}%", ema * 100.0));

    rsx! {
        tr {
            th { scope: "row", "Motor {motor}" }
            td { "{signal}" }
            td { "{pin}" }
            td {
                if visible {
                    {plot::gpio_plot(&state.history, pin, format!("Motor {motor} {signal} GPIO and EMA history"))}
                }
            }
            td {
                span { class: if level == Some(true) { "level high" } else { "level" }, "{level_text}" }
                div { class: "reading", output { "{ema}" } }
            }
            td {
                AlphaSlider {
                    label: format!("Motor {motor} {signal} EMA alpha"), value: alpha, disabled: channel.is_none(),
                    onchange: move |value| {
                        let saving = pending.peek().is_some();
                        pending.set(Some(value));
                        if saving { return; }
                        spawn(async move {
                            let mut value = value;
                            loop {
                                let saved = Request::put(&format!("/api/alpha/{pin}/{value}"))
                                    .header("x-gpio-monitor", "1")
                                    .send().await.is_ok_and(|response| response.ok());
                                if !saved {
                                    error.set(format!("Could not save alpha for GPIO {pin}."));
                                    break;
                                }
                                let next = *pending.peek();
                                if let Some(next) = next.filter(|next| *next != value) {
                                    value = next;
                                } else {
                                    if let Some(channel) = dashboard.write().snapshot.as_mut()
                                        .and_then(|snapshot| snapshot.channels.get_mut(pin)) {
                                        channel.alpha = value;
                                    }
                                    error.set(String::new());
                                    break;
                                }
                            }
                            pending.set(None);
                        });
                    },
                }
            }
        }
    }
}

#[component]
fn AlphaSlider(
    label: String,
    value: u8,
    #[props(default)] disabled: bool,
    onchange: EventHandler<u8>,
) -> Element {
    let text = format!("{:.2}", f64::from(value) / 100.0);
    rsx! {
        label { class: "alpha",
            input {
                r#type: "range", min: "1", max: "100", value: "{value}",
                aria_label: "{label}", aria_valuetext: "{text}", disabled,
                oninput: move |event| {
                    if let Ok(value) = event.value().parse::<u8>() {
                        onchange.call(value.clamp(1, 100));
                    }
                },
                onwheel: move |event| {
                    let delta = event.delta().strip_units().y;
                    if disabled || event.modifiers().contains(Modifiers::CONTROL) || delta == 0.0 {
                        return;
                    }
                    event.prevent_default();
                    onchange.call((i16::from(value) - delta.signum() as i16).clamp(1, 100) as u8);
                },
            }
            output { "{text}" }
        }
    }
}
