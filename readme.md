## Installation

### Rust

install rust from [here](https://www.rust-lang.org/tools/install)

Please use everything as default especially in Windows.

### Git

use `winget install git` in windows


## Cargo

Since we have private dependencies, we need to login to use git from system. Add following to `~/.cargo/config`:

``` toml
[net]
git-fetch-with-cli = true
```

Make sure you have valid ssh keys in your system and git account.

###  Use nightly version of Rust

Type `rustup default nightly` in command window 

### Install target

Type `rustup target add thumbv8m.main-none-eabihf` in command window



## Build

```bash
cargo build --features "stm32u575ci"
```
You should replace `stm32u575ci` with your target chip.

Currently we only support very few chips.

## Flash and Run 

You need `probe-rs` to flash and run the code. Install it with:

```bash
cargo install cargo-binstall
cargo binstall probe-rs
```

Or you can install with `cargo binstall probe-rs"

Refer to [probe-rs](https://probe.rs/) for more information.

Then we can flash and run with:

```bash
cargo run --features "stm32u575ci"
```







