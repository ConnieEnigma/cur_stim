## Development Environments

This repository supports three primary development workflows. Choose the one that best suits your needs.

### 1. GitHub Codespaces (Recommended)

For the most straightforward setup, we recommend using GitHub Codespaces. This provides a fully configured development environment in the cloud, accessible from your browser or VS Code.

To get started:

1.  Click the "Code" button on the GitHub repository page.
2.  Select the "Codespaces" tab.
3.  Click "Create codespace on main".

This will launch a new Codespace with all the necessary tools and dependencies pre-installed, allowing you to edit and build the project.

**Note:** Flashing and running the application on a physical device is not possible from GitHub Codespaces. This environment is best used for code editing and compilation.

### 2. Local Development (Native or Nix)

For building and running the project on a target device, you'll need a local setup. You can choose between a native environment or a Nix-managed environment.

#### Setup Option A: Native Environment

This approach involves setting up the required toolchain on your local operating system.

##### Rust

Install Rust from the official website: [https://www.rust-lang.org/tools/install](https://www.rust-lang.org/tools/install)

Please use the default settings during installation, especially on Windows.

##### Git

On Windows, you can install Git using winget:
`winget install git`

##### Use Stable Rust Toolchain

This project is compatible with the stable Rust toolchain, version 1.80 and later. After installing Rust, ensure you have the correct version:

`rustup default stable`
`rustc --version`

##### Install Target Architecture

Install the compilation target for the microcontroller:
`rustup target add thumbv8m.main-none-eabihf`

#### Setup Option B: Nix Environment

For a reproducible local setup, you can use the Nix package manager. The `flake.nix` file in this repository defines the exact development environment.

To activate the environment, run the following command in the project's root directory:

```bash
nix develop
```

This will drop you into a shell with the correct Rust toolchain and all other dependencies available, as defined in `flake.nix`.

## Building and Running (Local & Nix Environments)

These steps apply to both Native and Nix local environments.

### Building

Once your environment is set up, you can build the project with:

```bash
cargo build
```

### Flashing and Running

We use `probe-rs` to flash the firmware to the target device.

To flash and run the application, use:

```bash
cargo run
```
