pub mod types;
pub mod inset;
pub mod aisle_graph;
pub mod segment;
pub mod island;
pub mod clip;
pub mod face;
pub mod generate;

use wasm_bindgen::prelude::*;
use crate::generate::generate;
use crate::types::GenerateInput;

#[wasm_bindgen]
pub fn generate_js(input_json: &str) -> String {
    let input: GenerateInput = serde_json::from_str(input_json).unwrap();
    let layout = generate(input);
    serde_json::to_string(&layout).unwrap()
}
