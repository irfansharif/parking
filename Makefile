.PHONY: build-engine bindings dev test test-engine tsc clean

# Regenerate the TS type bindings (ts-rs-derived) consumed by the UI.
# Kept as its own target so `cargo test` isn't serialized behind the
# release-profile wasm-pack build. `build-engine` invokes it.
bindings:
	cd engine && TS_RS_EXPORT_DIR=$(CURDIR)/ui/src/bindings cargo test --lib export_bindings --quiet

build-engine: bindings
	cd engine && touch src/lib.rs && wasm-pack build --target web --release --out-dir ../ui/src/wasm

dev: build-engine
	cd ui && npx vite

# Rust-side tests: engine unit + harness fixtures. Fast; no wasm
# required.
test-engine:
	cd engine && cargo test

# TS-side type check. Catches drift between hand-written UI code and
# the regenerated bindings/wasm surface.
tsc: bindings
	cd ui && npx tsc --noEmit

test: test-engine tsc

clean:
	rm -rf engine/target ui/src/wasm ui/dist ui/node_modules
