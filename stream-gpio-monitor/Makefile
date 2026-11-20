WASM_BINDGEN ?= wasm-bindgen

.PHONY: dashboard
dashboard:
	cargo build --locked --release --target wasm32-unknown-unknown --target-dir dashboard/target --manifest-path dashboard/Cargo.toml
	$(WASM_BINDGEN) --target web --out-dir dashboard/dist --out-name dashboard dashboard/target/wasm32-unknown-unknown/release/spirob_dashboard.wasm
