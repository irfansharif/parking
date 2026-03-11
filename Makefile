.PHONY: build-engine dev test test-update clean

build-engine:
	cd engine && touch src/lib.rs && wasm-pack build --target web --release --out-dir ../ui/src/wasm

dev: build-engine
	cd ui && npx vite

test: build-engine
	cd ui && npx vite build
	npx playwright test

test-update: build-engine
	cd ui && npx vite build
	UPDATE_SNAPSHOTS=1 npx playwright test --update-snapshots $(if $(GREP),--grep "$(GREP)",)

clean:
	rm -rf engine/target ui/src/wasm ui/dist ui/node_modules
