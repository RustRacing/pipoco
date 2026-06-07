use std::{env, process};

use ecu_firmware_resolver::{
    resolve_bin_path, resolve_command, resolve_elf_path, resolve_firmware_selection,
    resolve_flash_command, resolve_objcopy_command, resolve_ts_asset_path,
    resolve_ts_generate_command, resolve_ts_generated_path, resolve_ts_ini, SUPPORTED_ALIASES,
    SUPPORTED_INVOCATIONS,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OutputMode {
    Command,
    ElfPath,
    BinPath,
    ObjcopyCommand,
    FlashCommand,
    TsAssetPath,
    TsGeneratedPath,
    TsGenerateCommand,
    TsPrintIni,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CliCommand<'a> {
    List,
    Resolve {
        mode: OutputMode,
        target: ResolveTarget<'a>,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ResolveTarget<'a> {
    BoardRecipe { board: &'a str, recipe: &'a str },
    Alias(&'a str),
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let Some(command) = parse_args(&args) else {
        eprintln!(
            "usage: {} --list | [--elf-path|--bin-path|--objcopy-command|--flash-command|--ts-asset-path|--ts-generated-path|--ts-generate-command|--ts-print-ini] (<board> <recipe> | <alias>)",
            args[0]
        );
        print_supported_stderr();
        process::exit(2);
    };
    let CliCommand::Resolve { mode, target } = command else {
        print_supported_stdout();
        return;
    };

    let selection = match target {
        ResolveTarget::BoardRecipe { board, recipe } => {
            resolve_firmware_selection(board, Some(recipe))
        }
        ResolveTarget::Alias(alias) => resolve_firmware_selection(alias, None),
    };
    let selection = match selection {
        Ok(selection) => selection,
        Err(error) => {
            eprintln!("error: {}: {error}", render_target(target));
            print_supported_stderr();
            process::exit(1);
        }
    };
    let board = selection.board;
    let recipe = selection.recipe;

    let rendered = match mode {
        OutputMode::Command => resolve_command(board, recipe),
        OutputMode::ElfPath => resolve_elf_path(board, recipe),
        OutputMode::BinPath => resolve_bin_path(board, recipe),
        OutputMode::ObjcopyCommand => resolve_objcopy_command(board, recipe),
        OutputMode::FlashCommand => resolve_flash_command(board, recipe),
        OutputMode::TsAssetPath => resolve_ts_asset_path(board, recipe),
        OutputMode::TsGeneratedPath => resolve_ts_generated_path(board, recipe),
        OutputMode::TsGenerateCommand => resolve_ts_generate_command(board, recipe),
        OutputMode::TsPrintIni => resolve_ts_ini(board, recipe),
    };

    match rendered {
        Ok(value) => println!("{value}"),
        Err(error) => {
            eprintln!("error: {}: {error}", render_target(target));
            print_supported_stderr();
            process::exit(1);
        }
    }
}

fn parse_args(args: &[String]) -> Option<CliCommand<'_>> {
    match args {
        [_, flag] if flag == "--list" => Some(CliCommand::List),
        [_, alias] => Some(make_alias_resolve_command(OutputMode::Command, alias)),
        [_, flag, alias] if flag == "--elf-path" => {
            Some(make_alias_resolve_command(OutputMode::ElfPath, alias))
        }
        [_, flag, board, recipe] if flag == "--elf-path" => {
            Some(make_resolve_command(OutputMode::ElfPath, board, recipe))
        }
        [_, flag, alias] if flag == "--bin-path" => {
            Some(make_alias_resolve_command(OutputMode::BinPath, alias))
        }
        [_, flag, board, recipe] if flag == "--bin-path" => {
            Some(make_resolve_command(OutputMode::BinPath, board, recipe))
        }
        [_, flag, alias] if flag == "--objcopy-command" => Some(make_alias_resolve_command(
            OutputMode::ObjcopyCommand,
            alias,
        )),
        [_, flag, board, recipe] if flag == "--objcopy-command" => Some(make_resolve_command(
            OutputMode::ObjcopyCommand,
            board,
            recipe,
        )),
        [_, flag, alias] if flag == "--flash-command" => {
            Some(make_alias_resolve_command(OutputMode::FlashCommand, alias))
        }
        [_, flag, board, recipe] if flag == "--flash-command" => Some(make_resolve_command(
            OutputMode::FlashCommand,
            board,
            recipe,
        )),
        [_, flag, alias] if flag == "--ts-asset-path" => {
            Some(make_alias_resolve_command(OutputMode::TsAssetPath, alias))
        }
        [_, flag, board, recipe] if flag == "--ts-asset-path" => {
            Some(make_resolve_command(OutputMode::TsAssetPath, board, recipe))
        }
        [_, flag, alias] if flag == "--ts-generated-path" => Some(make_alias_resolve_command(
            OutputMode::TsGeneratedPath,
            alias,
        )),
        [_, flag, board, recipe] if flag == "--ts-generated-path" => Some(make_resolve_command(
            OutputMode::TsGeneratedPath,
            board,
            recipe,
        )),
        [_, flag, alias] if flag == "--ts-generate-command" => Some(make_alias_resolve_command(
            OutputMode::TsGenerateCommand,
            alias,
        )),
        [_, flag, board, recipe] if flag == "--ts-generate-command" => Some(make_resolve_command(
            OutputMode::TsGenerateCommand,
            board,
            recipe,
        )),
        [_, flag, alias] if flag == "--ts-print-ini" => {
            Some(make_alias_resolve_command(OutputMode::TsPrintIni, alias))
        }
        [_, flag, board, recipe] if flag == "--ts-print-ini" => {
            Some(make_resolve_command(OutputMode::TsPrintIni, board, recipe))
        }
        [_, flag, _] if flag.starts_with("--") => None,
        [_, board, recipe] => Some(make_board_recipe_resolve_command(
            OutputMode::Command,
            board,
            recipe,
        )),
        _ => None,
    }
}

fn make_resolve_command<'a>(mode: OutputMode, board: &'a str, recipe: &'a str) -> CliCommand<'a> {
    make_board_recipe_resolve_command(mode, board, recipe)
}

fn make_board_recipe_resolve_command<'a>(
    mode: OutputMode,
    board: &'a str,
    recipe: &'a str,
) -> CliCommand<'a> {
    CliCommand::Resolve {
        mode,
        target: ResolveTarget::BoardRecipe { board, recipe },
    }
}

fn make_alias_resolve_command<'a>(mode: OutputMode, alias: &'a str) -> CliCommand<'a> {
    CliCommand::Resolve {
        mode,
        target: ResolveTarget::Alias(alias),
    }
}

fn render_target(target: ResolveTarget<'_>) -> String {
    match target {
        ResolveTarget::BoardRecipe { board, recipe } => format!("{board} {recipe}"),
        ResolveTarget::Alias(alias) => alias.to_string(),
    }
}

fn print_supported_stdout() {
    println!("supported invocations:");
    for (board, recipe) in SUPPORTED_INVOCATIONS {
        println!("  {board} {recipe}");
    }
    println!("aliases:");
    for alias in SUPPORTED_ALIASES {
        println!("  {} -> {} {}", alias.alias, alias.board, alias.recipe);
    }
}

fn print_supported_stderr() {
    eprintln!("supported invocations:");
    for (board, recipe) in SUPPORTED_INVOCATIONS {
        eprintln!("  {board} {recipe}");
    }
    eprintln!("aliases:");
    for alias in SUPPORTED_ALIASES {
        eprintln!("  {} -> {} {}", alias.alias, alias.board, alias.recipe);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CANONICAL_RECIPE: &str = "ignition-only-wasted-spark-no-watchdog-bringup";

    fn args(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| value.to_string()).collect()
    }

    #[test]
    fn parser_defaults_to_cargo_build_command() {
        let values = args(&["resolver", "rp2040-pico", CANONICAL_RECIPE]);

        assert_eq!(
            parse_args(&values),
            Some(CliCommand::Resolve {
                mode: OutputMode::Command,
                target: ResolveTarget::BoardRecipe {
                    board: "rp2040-pico",
                    recipe: CANONICAL_RECIPE
                }
            })
        );
    }

    #[test]
    fn parser_accepts_alias_for_default_command() {
        let values = args(&["resolver", "pico_ignition_only_wasted_spark"]);

        assert_eq!(
            parse_args(&values),
            Some(CliCommand::Resolve {
                mode: OutputMode::Command,
                target: ResolveTarget::Alias("pico_ignition_only_wasted_spark")
            })
        );
    }

    #[test]
    fn parser_accepts_list_mode_without_board_or_recipe() {
        let values = args(&["resolver", "--list"]);

        assert_eq!(parse_args(&values), Some(CliCommand::List));
    }

    #[test]
    fn parser_accepts_explicit_artifact_modes() {
        let elf = args(&["resolver", "--elf-path", "rp2040-pico", CANONICAL_RECIPE]);
        let bin = args(&["resolver", "--bin-path", "rp2040-pico", CANONICAL_RECIPE]);
        let objcopy = args(&[
            "resolver",
            "--objcopy-command",
            "rp2040-pico",
            CANONICAL_RECIPE,
        ]);
        let flash = args(&[
            "resolver",
            "--flash-command",
            "rp2040-pico",
            CANONICAL_RECIPE,
        ]);
        let ts_asset = args(&[
            "resolver",
            "--ts-asset-path",
            "rp2040-pico",
            CANONICAL_RECIPE,
        ]);
        let ts_generated = args(&[
            "resolver",
            "--ts-generated-path",
            "rp2040-pico",
            CANONICAL_RECIPE,
        ]);
        let ts_generate = args(&[
            "resolver",
            "--ts-generate-command",
            "rp2040-pico",
            CANONICAL_RECIPE,
        ]);
        let ts_print_ini = args(&[
            "resolver",
            "--ts-print-ini",
            "rp2040-pico",
            CANONICAL_RECIPE,
        ]);

        assert_eq!(
            parse_args(&elf),
            Some(CliCommand::Resolve {
                mode: OutputMode::ElfPath,
                target: ResolveTarget::BoardRecipe {
                    board: "rp2040-pico",
                    recipe: CANONICAL_RECIPE
                }
            })
        );
        assert_eq!(
            parse_args(&bin),
            Some(CliCommand::Resolve {
                mode: OutputMode::BinPath,
                target: ResolveTarget::BoardRecipe {
                    board: "rp2040-pico",
                    recipe: CANONICAL_RECIPE
                }
            })
        );
        assert_eq!(
            parse_args(&objcopy),
            Some(CliCommand::Resolve {
                mode: OutputMode::ObjcopyCommand,
                target: ResolveTarget::BoardRecipe {
                    board: "rp2040-pico",
                    recipe: CANONICAL_RECIPE
                }
            })
        );
        assert_eq!(
            parse_args(&flash),
            Some(CliCommand::Resolve {
                mode: OutputMode::FlashCommand,
                target: ResolveTarget::BoardRecipe {
                    board: "rp2040-pico",
                    recipe: CANONICAL_RECIPE
                }
            })
        );
        assert_eq!(
            parse_args(&ts_asset),
            Some(CliCommand::Resolve {
                mode: OutputMode::TsAssetPath,
                target: ResolveTarget::BoardRecipe {
                    board: "rp2040-pico",
                    recipe: CANONICAL_RECIPE
                }
            })
        );
        assert_eq!(
            parse_args(&ts_generated),
            Some(CliCommand::Resolve {
                mode: OutputMode::TsGeneratedPath,
                target: ResolveTarget::BoardRecipe {
                    board: "rp2040-pico",
                    recipe: CANONICAL_RECIPE
                }
            })
        );
        assert_eq!(
            parse_args(&ts_generate),
            Some(CliCommand::Resolve {
                mode: OutputMode::TsGenerateCommand,
                target: ResolveTarget::BoardRecipe {
                    board: "rp2040-pico",
                    recipe: CANONICAL_RECIPE
                }
            })
        );
        assert_eq!(
            parse_args(&ts_print_ini),
            Some(CliCommand::Resolve {
                mode: OutputMode::TsPrintIni,
                target: ResolveTarget::BoardRecipe {
                    board: "rp2040-pico",
                    recipe: CANONICAL_RECIPE
                }
            })
        );
    }

    #[test]
    fn parser_accepts_alias_for_flag_modes() {
        let values = args(&[
            "resolver",
            "--bin-path",
            "rp2040_pico_ignition_only_wasted_spark",
        ]);

        assert_eq!(
            parse_args(&values),
            Some(CliCommand::Resolve {
                mode: OutputMode::BinPath,
                target: ResolveTarget::Alias("rp2040_pico_ignition_only_wasted_spark")
            })
        );
    }

    #[test]
    fn parser_rejects_unknown_flags() {
        let values = args(&["resolver", "--flash", "rp2040-pico", CANONICAL_RECIPE]);

        assert_eq!(parse_args(&values), None);
    }
}
