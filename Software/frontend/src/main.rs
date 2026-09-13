use std::io::Write;

use clap::Parser;
use crossterm::{QueueableCommand, cursor, queue, terminal::Clear};
use inquire::{Text, validator::Validation};
use shared::remote_commands::COMMANDS;

#[derive(Parser)]
#[command()]
struct Args {
    tty: String,

    #[arg(short, long, default_value_t = 115200)]
    baud: u32,
}

fn main() {
    let args = Args::parse();
    let mut port = serialport::new(args.tty, args.baud)
        .open()
        .expect("Failed to open");

    let mut serial_buf = [0u8; 4096];

    loop {
        let cmd = Text::new("Command: ")
            .with_autocomplete(|input: &str| {
                Ok(COMMANDS
                    .iter()
                    .map(|c| c.0.to_string())
                    .filter(|c| c.starts_with(input))
                    .collect::<Vec<_>>())
            })
            .with_validator(|input: &str| {
                if COMMANDS.map(|c| c.0).contains(&input) {
                    Ok(Validation::Valid)
                } else {
                    Ok(Validation::Invalid(
                        format!("Invalid command: {}", input).into(),
                    ))
                }
            })
            .prompt();

        match cmd {
            Ok(mut cmd) => {
                cmd += "\n";
                port.write(cmd.as_bytes()).expect("write");
                let mut stdout = std::io::stdout();

                print!("Waiting for response...");
                stdout.flush().unwrap();

                while let Ok(n) = port.bytes_to_read()
                    && n == 0
                {}
                let n = port.read(&mut serial_buf).expect("read");
                queue!(
                    stdout,
                    Clear(crossterm::terminal::ClearType::CurrentLine),
                    cursor::MoveToColumn(0)
                )
                .unwrap();
                println!("{}", str::from_utf8(&serial_buf[0..n]).unwrap());
            }
            Err(e) => match e {
                inquire::InquireError::OperationInterrupted => return,
                inquire::InquireError::OperationCanceled => continue,
                _ => {}
            },
        }
    }
}
