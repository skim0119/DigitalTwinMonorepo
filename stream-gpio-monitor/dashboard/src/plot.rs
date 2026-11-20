use crate::{DEFAULT_ALPHA, GAP_MS, Point, WINDOW_MS};
use dioxus::prelude::*;
use std::fmt::Write;

pub fn noise_grams(points: &[Point], motor: usize) -> Option<f64> {
    let mut sum = 0.0;
    let mut count = 0;
    for point in points {
        if let Some((load, ema)) = point.kg[motor].zip(point.kg_ema[motor]) {
            sum += (load - ema).powi(2);
            count += 1;
        }
    }
    (count >= 2).then(|| 1000.0 * (sum / f64::from(count)).sqrt())
}

fn load_ema(points: &[Point], motor: usize, alpha: u8) -> Vec<Option<f64>> {
    if alpha == DEFAULT_ALPHA {
        return points.iter().map(|point| point.kg_ema[motor]).collect();
    }
    let alpha = f64::from(alpha) / 100.0;
    let mut previous = None;
    let mut average = None;
    points
        .iter()
        .map(|point| {
            if previous.is_some_and(|time| point.t - time > GAP_MS) {
                average = None;
            }
            previous = Some(point.t);
            average =
                point.kg[motor].map(|load| average.map_or(load, |old| old + alpha * (load - old)));
            average
        })
        .collect()
}

pub fn load_plot(points: &[Point], motor: usize, alpha: u8) -> Element {
    let ema = load_ema(points, motor, alpha);
    let raw: Vec<_> = points.iter().map(|point| point.kg[motor]).collect();
    let bounds = raw
        .iter()
        .chain(&ema)
        .flatten()
        .copied()
        .fold(None, |bounds, value| {
            Some(bounds.map_or((value, value), |(low, high): (f64, f64)| {
                (low.min(value), high.max(value))
            }))
        });
    let (chart, high, low) = if let Some((min, max)) = bounds {
        let mut low = (min * 1000.0).floor() / 1000.0;
        let mut high = (max * 1000.0).ceil() / 1000.0;
        if low == high {
            low -= 0.001;
            high += 0.001;
        }
        (
            plot(
                points,
                46.0,
                false,
                format!("Motor {motor} load and EMA history in kg"),
                |i, measured| {
                    (if measured { raw[i] } else { ema[i] })
                        .map(|value| (value - low) / (high - low))
                },
            ),
            format!("{high:.3}"),
            format!("{low:.3}"),
        )
    } else {
        (
            rsx! { svg { class: "plot", role: "img", "aria-label": "Motor {motor} load and EMA history in kg" } },
            "—".into(),
            "—".into(),
        )
    };
    rsx! { div { class: "load-plot", {chart} div { class: "axis", span { "{high}" } span { "{low}" } } } }
}

pub fn gpio_plot(points: &[Point], pin: usize, label: String) -> Element {
    plot(points, 36.0, true, label, |i, raw| {
        let point = &points[i];
        Some(if raw {
            f64::from((point.levels >> pin) & 1)
        } else {
            f64::from(point.ema[pin]) / 100.0
        })
    })
}

fn plot(
    points: &[Point],
    height: f64,
    stepped: bool,
    label: String,
    value: impl Fn(usize, bool) -> Option<f64>,
) -> Element {
    let y = |sample: f64| height - 2.5 - sample * (height - 6.0);
    let grid = format!(
        "M0 {:.1}H1000 M0 {:.1}H1000 M0 {:.1}H1000 M500 3V{:.1}",
        y(0.0),
        y(0.5),
        y(1.0),
        height - 3.0
    );
    let trace = |raw| {
        let mut path = String::new();
        let Some(end) = points.last().map(|point| point.t) else {
            return path;
        };
        let mut previous: Option<(u64, f64)> = None;
        for (i, point) in points.iter().enumerate() {
            let Some(sample) = value(i, raw) else {
                previous = None;
                continue;
            };
            let x = (1.0 - (end - point.t) as f64 / WINDOW_MS as f64) * 998.0;
            let y = y(sample);
            if let Some((_, last_y)) = previous.filter(|(time, _)| point.t - time <= GAP_MS) {
                if raw && stepped {
                    let _ = write!(path, "L{x:.2} {last_y:.2}");
                }
            } else {
                let _ = write!(path, "M{:.2} {y:.2}", x - 2.0);
            }
            let _ = write!(path, "L{x:.2} {y:.2}");
            previous = Some((point.t, y));
        }
        path
    };
    let raw = trace(true);
    let ema = trace(false);
    rsx! {
        svg { class: "plot", role: "img", "aria-label": "{label}", view_box: "0 0 1000 {height}", preserve_aspect_ratio: "none",
            path { d: "{grid}", fill: "none", stroke: "#d5e0d9", vector_effect: "non-scaling-stroke" }
            path { d: "{raw}", fill: "none", stroke: "#274a6a", stroke_width: "1.5", vector_effect: "non-scaling-stroke" }
            path { d: "{ema}", fill: "none", stroke: "#a45808", stroke_width: "1.7", stroke_dasharray: "4 3", vector_effect: "non-scaling-stroke" }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adjusts_load_ema_and_keeps_noise_on_the_reference() {
        let points: Vec<_> = [
            (0, Some(0.0), Some(0.0)),
            (50, Some(0.004), Some(0.001)),
            (100, None, None),
            (150, Some(0.008), Some(0.008)),
            (500, Some(0.01), Some(0.01)),
        ]
        .into_iter()
        .map(|(t, load, ema)| Point {
            id: t,
            t,
            levels: 0,
            ema: [0; 28],
            kg: [load; 6],
            kg_ema: [ema; 6],
        })
        .collect();
        assert_eq!(
            load_ema(&points, 0, 50),
            vec![Some(0.0), Some(0.002), None, Some(0.008), Some(0.01)]
        );
        assert_eq!(noise_grams(&points, 0), Some(1.5));
    }
}
