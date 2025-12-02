

fn main() {
    // println!("cargo:rustc-link-arg-bins=-Tmemory.x"); // feagure in cortex-m-rt
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}


