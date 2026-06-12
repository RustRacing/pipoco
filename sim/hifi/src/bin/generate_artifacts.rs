use std::path::PathBuf;

fn main() {
    let output_dir = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("sim/hifi/artifacts"));

    ecu_sim_hifi::write_default_artifacts(&output_dir).unwrap_or_else(|error| {
        panic!(
            "failed to generate artifacts in {}: {error:?}",
            output_dir.display()
        )
    });

    println!("wrote hifi artifacts to {}", output_dir.display());
}
