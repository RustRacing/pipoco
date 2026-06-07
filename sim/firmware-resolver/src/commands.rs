use ecu_board_profiles::{FirmwareBuildInvocation, FirmwareBuildMode};

use crate::{selection, ResolveError};

pub fn resolve_command(board: &str, recipe: &str) -> Result<String, ResolveError> {
    let invocation = selection::resolve_invocation(board, recipe)?;
    Ok(render_cargo_build_command(invocation))
}

pub fn resolve_elf_path(board: &str, recipe: &str) -> Result<String, ResolveError> {
    let invocation = selection::resolve_invocation(board, recipe)?;
    Ok(render_elf_path(invocation))
}

pub fn resolve_bin_path(board: &str, recipe: &str) -> Result<String, ResolveError> {
    let elf = resolve_elf_path(board, recipe)?;
    Ok(format!("{elf}.bin"))
}

pub fn resolve_objcopy_command(board: &str, recipe: &str) -> Result<String, ResolveError> {
    let elf = resolve_elf_path(board, recipe)?;
    let bin = resolve_bin_path(board, recipe)?;
    Ok(ShellCommand::new(["rust-objcopy", "-O", "binary"])
        .arg(elf)
        .arg(bin)
        .render())
}

pub fn resolve_flash_command(board: &str, recipe: &str) -> Result<String, ResolveError> {
    let (_board_id, invocation) = selection::resolve_invocation_with_board(board, recipe)?;
    let elf = render_elf_path(invocation);
    let chip = selection::resolve_flash_chip(board, recipe)?;
    Ok(ShellCommand::new(["probe-rs", "run", "--chip", chip])
        .arg(elf)
        .render())
}

fn render_cargo_build_command(invocation: FirmwareBuildInvocation) -> String {
    let mut command = ShellCommand::new(["cargo", "build", "-p"])
        .arg(invocation.package)
        .arg("--bin")
        .arg(invocation.binary)
        .arg("--target")
        .arg(invocation.target_triple);
    if matches!(invocation.mode, FirmwareBuildMode::Release) {
        command = command.arg("--release");
    }
    if !invocation.features.is_empty() {
        command = command
            .arg("--features")
            .arg(invocation.features.as_slice().join(","));
    }
    command.render()
}

fn render_elf_path(invocation: FirmwareBuildInvocation) -> String {
    let profile = match invocation.mode {
        FirmwareBuildMode::Debug => "debug",
        FirmwareBuildMode::Release => "release",
    };
    format!(
        "target/{}/{}/{}",
        invocation.target_triple, profile, invocation.binary
    )
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ShellCommand {
    args: Vec<String>,
}

impl ShellCommand {
    fn new<const N: usize>(args: [&str; N]) -> Self {
        let mut command = Self { args: Vec::new() };
        for arg in args {
            command = command.arg(arg);
        }
        command
    }

    fn arg(mut self, arg: impl Into<String>) -> Self {
        let arg = arg.into();
        assert_valid_display_arg(&arg);
        self.args.push(arg);
        self
    }

    fn render(self) -> String {
        self.args.join(" ")
    }
}

fn assert_valid_display_arg(arg: &str) {
    assert!(
        !arg.is_empty(),
        "firmware resolver command argument must not be empty"
    );
    assert!(
        arg.bytes().all(is_safe_display_arg_byte),
        "firmware resolver command argument contains shell-sensitive characters: {arg:?}"
    );
}

fn is_safe_display_arg_byte(byte: u8) -> bool {
    matches!(
        byte,
        b'a'..=b'z'
            | b'A'..=b'Z'
            | b'0'..=b'9'
            | b'-'
            | b'_'
            | b'.'
            | b'/'
            | b','
            | b':'
            | b'='
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rendered_commands_use_safe_display_arguments() {
        let command = resolve_command(
            "rp2040-pico",
            "ignition-only-wasted-spark-no-watchdog-bringup",
        )
        .unwrap();
        for arg in command.split(' ') {
            assert_valid_display_arg(arg);
        }
    }

    #[test]
    #[should_panic(expected = "shell-sensitive characters")]
    fn command_display_args_reject_shell_sensitive_characters() {
        let _ = ShellCommand::new(["cargo"]).arg("bad;rm");
    }

    #[test]
    fn flash_command_uses_fixture_chip_mapping() {
        let command = resolve_flash_command(
            "rp2040-pico",
            "ignition-only-wasted-spark-no-watchdog-bringup",
        )
        .unwrap();
        assert_eq!(
            command,
            "probe-rs run --chip RP2040 target/thumbv6m-none-eabi/release/ts-ecu"
        );

        let command = resolve_flash_command("rp2350b", "rev-limiter-no-watchdog-bringup").unwrap();
        assert_eq!(
            command,
            "probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/ecu-rp2350b-min"
        );
    }

    #[test]
    fn flash_command_missing_chip_is_rejected() {
        assert!(matches!(
            resolve_flash_command("stm32f4", "ignition-only-wasted-spark-no-watchdog-bringup"),
            Err(ResolveError::NoFlashCommand)
        ));
    }
}
