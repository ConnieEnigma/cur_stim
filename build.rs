
use std::env;
use std::fs::File;
use std::io::Write;

fn main() {
    // println!("cargo:rustc-link-arg-bins=-Tmemory.x"); // feagure in cortex-m-rt
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    let runner = if cfg!(feature = "stm32u5a5zj") {
        "probe-rs run --chip STM32U5A5ZJTx"
    } else if cfg!(feature = "stm32u575zi") {
        "probe-rs run --chip STM32U575ZITxQ"
    } else if cfg!(feature = "stm32u575ci") {
        "probe-rs run --chip STM32U575CIUxQ"
    } else {
        panic!("No chip selected");
    };
    // let config_path = Path::new(".cargo").join("config.toml");
    // let mut config_file = File::create(&config_path).expect("Failed to create config file");
    //
    // writeln!(config_file, "[target.thumbv8m.main-none-eabihf]").expect("Failed to write to config file");
    // writeln!(config_file, "runner = \"{}\"", runner).expect("Failed to write to config file");
    //
    // writeln!(config_file, "[build]").expect("Failed to write to config file");
    // writeln!(config_file, "target = \"thumbv8m.main-none-eabihf\"").expect("Failed to write to config file");
    //
    // writeln!(config_file, "[env]").expect("Failed to write to config file");
    // writeln!(config_file, "DEFMT_LOG = \"info\"").expect("Failed to write to config file");
    // // writeln!(config_file, "DEFMT_TIMESTAMP = \"1\"").expect("Failed to write to config file");
    //
    // env::set_var("CARGO_RUNNER", runner);
}


