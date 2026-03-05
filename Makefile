.PHONY: build-engine dev test test-update clean

build-engine:
	cd engine && wasm-pack build --target web --release --out-dir ../ui/src/wasm

dev: build-engine
	cd ui && npx vite

test: build-engine
	cd ui && npx vite build
	npx playwright test

test-update: build-engine
	cd ui && npx vite build
	npx playwright test --update-snapshots

clean:
	rm -rf engine/target ui/src/wasm ui/dist ui/node_modules
