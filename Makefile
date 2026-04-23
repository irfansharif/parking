.PHONY: build-engine bindings dev test test-engine test-ui test-update tsc clean

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

# Full UI test run: browser-side Playwright fixtures (the one
# remaining fixture covers render-layer toggles that engine tests
# can't).
test-ui: build-engine
	cd ui && npx vite build
	npx playwright test

# Belt-and-suspenders: run every check in the repo.
test: test-engine tsc test-ui

test-update: build-engine
	cd ui && npx vite build
	UPDATE_SNAPSHOTS=1 npx playwright test --update-snapshots $(if $(GREP),--grep "$(GREP)",)

clean:
	rm -rf engine/target ui/src/wasm ui/dist ui/node_modules
