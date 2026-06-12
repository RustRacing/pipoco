use std::process::Command;

#[derive(Debug)]
struct PackageMetadata {
    name: String,
    normal_dependencies: Vec<String>,
    dev_dependencies: Vec<String>,
    build_dependencies: Vec<String>,
}

#[test]
fn hifi_dependencies_stay_within_the_adr_boundary() {
    let metadata = cargo_metadata();
    let hifi = metadata
        .iter()
        .find(|package| package.name == "ecu-sim-hifi")
        .expect("ecu-sim-hifi package present");

    for dependency in &hifi.normal_dependencies {
        assert!(
            matches!(dependency.as_str(), "ecu-domain" | "ecu-io"),
            "ecu-sim-hifi gained forbidden normal dependency {dependency}",
        );
    }

    for dependency in &hifi.dev_dependencies {
        assert!(
            matches!(dependency.as_str(), "ecu-domain" | "ecu-io"),
            "ecu-sim-hifi gained forbidden dev dependency {dependency}",
        );
    }

    for dependency in &hifi.build_dependencies {
        assert!(
            matches!(dependency.as_str(), "ecu-domain" | "ecu-io"),
            "ecu-sim-hifi gained forbidden build dependency {dependency}",
        );
    }

    let reverse_dependents: Vec<&str> = metadata
        .iter()
        .filter(|package| package.name != "ecu-sim-hifi" && package_depends_on_hifi(package))
        .map(|package| package.name.as_str())
        .collect();

    for dependent in reverse_dependents {
        assert!(
            dependent == "ecu-sim-driver",
            "forbidden reverse dependency on ecu-sim-hifi from {dependent}",
        );
    }
}

fn package_depends_on_hifi(package: &PackageMetadata) -> bool {
    package
        .normal_dependencies
        .iter()
        .chain(package.dev_dependencies.iter())
        .chain(package.build_dependencies.iter())
        .any(|dependency| dependency == "ecu-sim-hifi")
}

fn cargo_metadata() -> Vec<PackageMetadata> {
    let output = Command::new("cargo")
        .args(["metadata", "--format-version", "1", "--no-deps"])
        .output()
        .expect("cargo metadata command should run");
    assert!(
        output.status.success(),
        "cargo metadata failed: {}",
        String::from_utf8_lossy(&output.stderr),
    );

    parse_packages(&String::from_utf8(output.stdout).expect("metadata should be utf-8"))
}

fn parse_packages(json: &str) -> Vec<PackageMetadata> {
    let packages_payload = array_field(json, "packages").expect("packages array present");
    split_top_level_objects(packages_payload)
        .into_iter()
        .map(parse_package)
        .collect()
}

fn parse_package(object: &str) -> PackageMetadata {
    let dependencies = array_field(object, "dependencies").expect("dependencies array present");

    let mut normal_dependencies = Vec::new();
    let mut dev_dependencies = Vec::new();
    let mut build_dependencies = Vec::new();

    for dep in split_top_level_objects(dependencies) {
        let name = string_field(dep, "name").expect("dependency name present");
        let kind = parse_dependency_kind(raw_field(dep, "kind").expect("dependency kind present"));

        match kind {
            Some(DependencyKind::Normal) => normal_dependencies.push(name),
            Some(DependencyKind::Dev) => dev_dependencies.push(name),
            Some(DependencyKind::Build) => build_dependencies.push(name),
            Some(DependencyKind::Unknown) | None => {}
        }
    }

    PackageMetadata {
        name: string_field(object, "name").expect("package name present"),
        normal_dependencies,
        dev_dependencies,
        build_dependencies,
    }
}

#[derive(Debug)]
enum DependencyKind {
    Normal,
    Dev,
    Build,
    Unknown,
}

fn parse_dependency_kind(raw: &str) -> Option<DependencyKind> {
    let trimmed = raw.trim();
    Some(match trimmed {
        "null" => DependencyKind::Normal,
        "\"dev\"" => DependencyKind::Dev,
        "\"build\"" => DependencyKind::Build,
        _ => DependencyKind::Unknown,
    })
}

fn array_field<'a>(object: &'a str, field: &str) -> Option<&'a str> {
    let field_key = format!("\"{field}\":");
    let start = object.find(&field_key)? + field_key.len();
    let array_start = object[start..].find('[')? + start;
    let array_end = matching_delimiter(object, array_start, '[', ']')?;
    Some(&object[array_start + 1..array_end])
}

fn string_field(object: &str, field: &str) -> Option<String> {
    let field_key = format!("\"{field}\":\"");
    let start = object.find(&field_key)? + field_key.len();
    let mut escaped = false;
    let mut value = String::new();
    for ch in object[start..].chars() {
        if escaped {
            value.push(ch);
            escaped = false;
            continue;
        }
        match ch {
            '\\' => escaped = true,
            '"' => return Some(value),
            _ => value.push(ch),
        }
    }
    None
}

fn raw_field<'a>(object: &'a str, field: &str) -> Option<&'a str> {
    let field_key = format!("\"{field}\":");
    let start = object.find(&field_key)? + field_key.len();
    let rest = &object[start..];
    let end = rest
        .find(',')
        .unwrap_or_else(|| rest.find('}').expect("field terminator present"));
    Some(rest[..end].trim())
}

fn split_top_level_objects(payload: &str) -> Vec<&str> {
    let mut items = Vec::new();
    let mut in_string = false;
    let mut escaped = false;
    let mut depth = 0_i32;
    let mut object_start = None;

    for (index, ch) in payload.char_indices() {
        if in_string {
            if escaped {
                escaped = false;
            } else if ch == '\\' {
                escaped = true;
            } else if ch == '"' {
                in_string = false;
            }
            continue;
        }

        match ch {
            '"' => in_string = true,
            '{' => {
                if depth == 0 {
                    object_start = Some(index);
                }
                depth += 1;
            }
            '}' => {
                depth -= 1;
                if depth == 0 {
                    let start = object_start.expect("object start tracked");
                    items.push(&payload[start..=index]);
                }
            }
            _ => {}
        }
    }

    items
}

fn matching_delimiter(source: &str, start: usize, open: char, close: char) -> Option<usize> {
    let mut in_string = false;
    let mut escaped = false;
    let mut depth = 0_i32;

    for (offset, ch) in source[start..].char_indices() {
        if in_string {
            if escaped {
                escaped = false;
            } else if ch == '\\' {
                escaped = true;
            } else if ch == '"' {
                in_string = false;
            }
            continue;
        }

        match ch {
            '"' => in_string = true,
            c if c == open => depth += 1,
            c if c == close => {
                depth -= 1;
                if depth == 0 {
                    return Some(start + offset);
                }
            }
            _ => {}
        }
    }

    None
}
