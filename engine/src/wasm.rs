//! `#[wasm_bindgen]` facade. The only place the engine talks to
//! JavaScript; all engine logic is native-testable without wasm-pack.

use wasm_bindgen::prelude::*;

use crate::debug::format_fixture;
use crate::pipeline::generate::generate;
use crate::types::GenerateInput;

#[wasm_bindgen]
pub fn generate_js(input_json: &str) -> String {
    let input: GenerateInput = serde_json::from_str(input_json).unwrap();
    let layout = generate(input);
    serde_json::to_string(&layout).unwrap()
}

/// Serialize a `GenerateInput` JSON into the datadriven test fixture format
/// used by `tests/testdata/*.txt`. Call from the browser console:
///   copy(debug_input_js(window.__parkingInput))
/// Then paste into a new .txt file under `tests/testdata/`.
#[wasm_bindgen]
pub fn debug_input_js(input_json: &str) -> String {
    let input: GenerateInput = serde_json::from_str(input_json).unwrap();
    format_fixture(&input)
}
