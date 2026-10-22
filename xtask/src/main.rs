use std::env;
use std::error::Error;
use std::ffi::OsString;
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::Duration;

use serde::Deserialize;
use serde_json::Value;

const REVISION: &str = include_str!("../../tests/assets/nmos6502/REVISION");
const OPCODES: &str = include_str!("../../tests/assets/nmos6502/opcodes.json");
const CASES_PER_OPCODE: usize = 10_000;
const SAMPLE_CASES_PER_OPCODE: usize = 8;
const USAGE: &str = "Usage: cargo xtask fetch-nmos [--sample-output PATH]";
type Result<T> = std::result::Result<T, Box<dyn Error>>;

// Validate the same field types as the test harness, but retain the original
// JSON values for sample output (including any additional upstream fields).
#[derive(Deserialize)]
#[allow(dead_code)]
struct State {
    pc: u16,
    s: u8,
    a: u8,
    x: u8,
    y: u8,
    p: u8,
    ram: Vec<(u16, u8)>,
}

#[derive(Deserialize)]
#[allow(dead_code)]
struct Case {
    name: String,
    initial: State,
    #[serde(rename = "final")]
    final_state: State,
    cycles: Vec<(u16, u8, String)>,
}

fn parse_args(args: impl IntoIterator<Item = OsString>) -> Result<Option<PathBuf>> {
    let mut args = args.into_iter();
    if args.next().as_deref() != Some(std::ffi::OsStr::new("fetch-nmos")) {
        return Err(USAGE.into());
    }
    let sample = match args.next() {
        None => return Ok(None),
        Some(flag) if flag == "--sample-output" => args
            .next()
            .filter(|path| !path.is_empty())
            .map(PathBuf::from)
            .ok_or(USAGE)?,
        _ => return Err(USAGE.into()),
    };
    if args.next().is_some() {
        return Err(USAGE.into());
    }
    Ok(Some(sample))
}

fn fixture_dir(root: &Path, override_path: Option<OsString>) -> PathBuf {
    override_path.map_or_else(|| root.join("target/nmos6502/v1"), PathBuf::from)
}

fn prepare_cache(dir: &Path) -> Result<()> {
    fs::create_dir_all(dir)?;
    let marker = dir.join("REVISION");
    match fs::read_to_string(&marker) {
        Ok(revision) if revision.trim() == REVISION.trim() => Ok(()),
        Ok(_) => Err(format!("Wrong NMOS fixture revision in {}", dir.display()).into()),
        Err(error) if error.kind() == io::ErrorKind::NotFound => {
            for entry in fs::read_dir(dir)? {
                if entry?.path().extension().is_some_and(|ext| ext == "json") {
                    return Err(format!(
                        "Fixtures without REVISION in {}; use a fresh NMOS6502_TESTS directory",
                        dir.display()
                    )
                    .into());
                }
            }
            fs::write(marker, REVISION)?;
            Ok(())
        }
        Err(error) => Err(error.into()),
    }
}

fn parse_fixture(data: &[u8], expected_opcode: u8) -> Result<Vec<Value>> {
    let cases: Vec<Value> = serde_json::from_slice(data)?;
    if cases.len() != CASES_PER_OPCODE {
        return Err(format!("Expected {CASES_PER_OPCODE} cases, got {}", cases.len()).into());
    }
    for (index, value) in cases.iter().enumerate() {
        let case = Case::deserialize(value).map_err(|error| format!("Case {index}: {error}"))?;
        let mut at_pc = case
            .initial
            .ram
            .iter()
            .filter(|(addr, _)| *addr == case.initial.pc);
        if at_pc.next().map(|(_, byte)| *byte) != Some(expected_opcode)
            || at_pc.any(|(_, byte)| *byte != expected_opcode)
        {
            return Err(
                format!("Case {index}: wrong or missing opcode {expected_opcode:02x}").into(),
            );
        }
    }
    Ok(cases)
}

fn load_fixture(client: &reqwest::blocking::Client, dir: &Path, opcode: u8) -> Result<Vec<Value>> {
    let path = dir.join(format!("{opcode:02x}.json"));
    match fs::read(&path) {
        Ok(data) => match parse_fixture(&data, opcode) {
            Ok(cases) => return Ok(cases),
            Err(error) => eprintln!("{}: {error}; downloading again", path.display()),
        },
        Err(error) if error.kind() == io::ErrorKind::NotFound => {}
        Err(error) => return Err(error.into()),
    }
    let url = format!(
        "https://raw.githubusercontent.com/SingleStepTests/65x02/{}/6502/v1/{opcode:02x}.json",
        REVISION.trim()
    );
    eprintln!("Downloading {url}");
    let data = client.get(&url).send()?.error_for_status()?.bytes()?;
    let cases = parse_fixture(&data, opcode).map_err(|error| format!("{url}: {error}"))?;
    fs::write(&path, &data)?;
    Ok(cases)
}

fn write_sample(mut output: impl Write, cases: &[Value]) -> Result<()> {
    writeln!(output, "[")?;
    for (index, case) in cases.iter().enumerate() {
        serde_json::to_writer(&mut output, case)?;
        writeln!(
            output,
            "{}",
            if index + 1 == cases.len() { "" } else { "," }
        )?;
    }
    writeln!(output, "]")?;
    Ok(())
}

fn run() -> Result<()> {
    let sample_output = parse_args(env::args_os().skip(1))?;
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .ok_or("Missing project root")?;
    let dir = fixture_dir(root, env::var_os("NMOS6502_TESTS"));
    let opcodes: Vec<u8> = serde_json::from_str(OPCODES)?;
    if opcodes.len() != 151 || !opcodes.windows(2).all(|pair| pair[0] < pair[1]) {
        return Err("Expected 151 sorted, distinct documented opcodes".into());
    }
    prepare_cache(&dir)?;
    let client = reqwest::blocking::Client::builder()
        .https_only(true)
        .connect_timeout(Duration::from_secs(30))
        .timeout(Duration::from_secs(120))
        .build()?;
    let mut sample = Vec::new();
    for opcode in &opcodes {
        let cases = load_fixture(&client, &dir, *opcode)?;
        if sample_output.is_some() {
            sample.extend(cases.into_iter().take(SAMPLE_CASES_PER_OPCODE));
        }
    }
    if let Some(path) = sample_output {
        let mut data = Vec::new();
        write_sample(&mut data, &sample)?;
        fs::write(&path, data)?;
        eprintln!("Wrote {} sample cases to {}", sample.len(), path.display());
    }
    eprintln!(
        "Validated {} opcode fixtures in {}",
        opcodes.len(),
        dir.display()
    );
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    struct TempDir(PathBuf);

    impl TempDir {
        fn new() -> Self {
            static NEXT: AtomicUsize = AtomicUsize::new(0);
            let path = env::temp_dir().join(format!(
                "mos6502-xtask-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
            fs::create_dir(&path).unwrap();
            Self(path)
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    fn case() -> Value {
        let sample: Vec<Value> =
            serde_json::from_str(include_str!("../../tests/assets/nmos6502/sample.json")).unwrap();
        sample[0].clone()
    }

    fn fixture(case: &Value) -> Vec<u8> {
        serde_json::to_vec(&vec![case; CASES_PER_OPCODE]).unwrap()
    }

    #[test]
    fn validates_count_schema_and_every_opcode() {
        let valid = case();
        assert!(parse_fixture(&fixture(&valid), 0x00).is_ok());
        assert!(parse_fixture(b"not json", 0x00).is_err());
        assert!(parse_fixture(b"{}", 0x00).is_err());
        assert!(parse_fixture(b"[]", 0x00).is_err());
        assert!(parse_fixture(&fixture(&valid), 0xa9).is_err());
        let mut cases = vec![valid.clone(); CASES_PER_OPCODE];
        cases[CASES_PER_OPCODE - 1]["initial"]["ram"] = serde_json::json!([]);
        assert!(parse_fixture(&serde_json::to_vec(&cases).unwrap(), 0x00).is_err());
        let mut invalid = valid.clone();
        invalid["final"]["a"] = serde_json::json!(256);
        assert!(parse_fixture(&fixture(&invalid), 0x00).is_err());
        let mut invalid = valid;
        invalid.as_object_mut().unwrap().remove("cycles");
        assert!(parse_fixture(&fixture(&invalid), 0x00).is_err());
    }

    #[test]
    fn revision_guard_and_cache_reuse() {
        let dir = TempDir::new();
        fs::write(dir.0.join("00.json"), fixture(&case())).unwrap();
        assert!(prepare_cache(&dir.0).is_err());
        assert!(!dir.0.join("REVISION").exists());
        fs::write(dir.0.join("REVISION"), "wrong").unwrap();
        assert!(prepare_cache(&dir.0).is_err());
        fs::write(dir.0.join("REVISION"), REVISION).unwrap();
        prepare_cache(&dir.0).unwrap();
        // Valid cached files must be reused without a network request.
        let client = reqwest::blocking::Client::new();
        assert_eq!(
            load_fixture(&client, &dir.0, 0x00).unwrap().len(),
            CASES_PER_OPCODE
        );
        assert_eq!(fs::read_dir(&dir.0).unwrap().count(), 2);
        let fresh = TempDir::new();
        prepare_cache(&fresh.0).unwrap();
        assert_eq!(
            fs::read_to_string(fresh.0.join("REVISION")).unwrap(),
            REVISION
        );
    }

    #[test]
    fn sample_preserves_fields_order_and_compact_lines() {
        let mut cases = vec![case(); 10];
        for (index, case) in cases.iter_mut().enumerate() {
            case["extra"] = serde_json::json!({"index": index, "text": "a\nb"});
        }
        let sample: Vec<_> = cases.into_iter().take(SAMPLE_CASES_PER_OPCODE).collect();
        let mut output = Vec::new();
        write_sample(&mut output, &sample).unwrap();
        assert_eq!(
            serde_json::from_slice::<Vec<Value>>(&output).unwrap(),
            sample
        );
        assert_eq!(
            String::from_utf8(output).unwrap().lines().count(),
            SAMPLE_CASES_PER_OPCODE + 2
        );
    }

    #[test]
    fn cli_and_cache_paths() {
        let parse = |args: &[&str]| parse_args(args.iter().map(OsString::from));
        assert_eq!(parse(&["fetch-nmos"]).unwrap(), None);
        assert_eq!(
            parse(&["fetch-nmos", "--sample-output", "some path.json"]).unwrap(),
            Some(PathBuf::from("some path.json"))
        );
        for args in [
            vec![],
            vec!["unknown"],
            vec!["fetch-nmos", "--sample-output"],
            vec!["fetch-nmos", "--unknown"],
            vec!["fetch-nmos", "--sample-output", "out", "extra"],
        ] {
            assert!(parse(&args).is_err());
        }
        assert_eq!(
            fixture_dir(Path::new("repo"), None),
            Path::new("repo/target/nmos6502/v1")
        );
        assert_eq!(
            fixture_dir(Path::new("repo"), Some("custom".into())),
            Path::new("custom")
        );
    }
}
