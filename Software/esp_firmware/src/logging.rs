use core::fmt::Arguments;

use defmt::{error, info, warn};
use fatfs::{File, LossyOemCpConverter, NullTimeProvider, Write};

use crate::{now_ms, sd::SdHost};

extern crate alloc;

use alloc::format;

type SdFile<'a> = File<'a, SdHost<'a>, NullTimeProvider, LossyOemCpConverter>;

#[derive(Default)]
pub struct Logger<'a> {
    file: Option<SdFile<'a>>,
}

impl<'a> Logger<'a> {
    pub fn with_file(&mut self, file: SdFile<'a>) {
        self.file = Some(file);
    }
    pub fn info(&mut self, args: Arguments<'_>) {
        let mut logged_to_file = false;
        if let Some(file) = self.file.as_mut() {
            logged_to_file = file
                .write(format!("[INFO] ({}) {}\n", now_ms!(), args).as_bytes())
                .is_ok();

            #[allow(unused_must_use)]
            file.flush();
        }

        if cfg!(feature = "logger_print_both") || !logged_to_file {
            info!("({}) {}", now_ms!(), alloc::fmt::format(args).as_str());
        }
    }
    pub fn warn(&mut self, args: Arguments<'_>) {
        let mut logged_to_file = false;
        if let Some(file) = self.file.as_mut() {
            logged_to_file = file
                .write(format!("[WARN] ({}) {}\n", now_ms!(), args).as_bytes())
                .is_ok();
            #[allow(unused_must_use)]
            file.flush();
        }

        if cfg!(feature = "logger_print_both") || !logged_to_file {
            warn!("({}) {}", now_ms!(), alloc::fmt::format(args).as_str());
        }
    }
    pub fn error(&mut self, args: Arguments<'_>) {
        let mut logged_to_file = false;
        if let Some(file) = self.file.as_mut() {
            logged_to_file = file
                .write(format!("[ERROR] ({}) {}\n", now_ms!(), args).as_bytes())
                .is_ok();
            #[allow(unused_must_use)]
            file.flush();
        }

        if cfg!(feature = "logger_print_both") || !logged_to_file {
            error!("({}) {}", now_ms!(), alloc::fmt::format(args).as_str());
        }
    }
}
