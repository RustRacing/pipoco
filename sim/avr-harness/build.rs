use std::{
    env,
    path::{Path, PathBuf},
    process::Command,
};

fn main() {
    let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let source = manifest_dir.join("firmware/mega2560_speeduino_m5x_stub.c");
    let elf = out_dir.join("mega2560_speeduino_m5x_stub.elf");

    println!("cargo:rerun-if-changed={}", source.display());
    compile_firmware(&source, &elf);
    println!("cargo:rustc-env=PIPOCO_AVR_STUB_ELF={}", elf.display());
}

fn compile_firmware(source: &Path, elf: &Path) {
    let status = Command::new("avr-gcc")
        .args([
            "-mmcu=atmega2560",
            "-DF_CPU=16000000UL",
            "-Os",
            "-std=gnu11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-o",
        ])
        .arg(elf)
        .arg(source)
        .status()
        .expect("failed to run avr-gcc; install gcc-avr and avr-libc");

    if !status.success() {
        panic!("avr-gcc failed while building {}", source.display());
    }
}
