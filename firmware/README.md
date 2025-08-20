```
cargo install just cargo-binutils

sudo apt update

sudo apt install teensy-loader-cli 

sudo apt install minicom

rustup component add llvm-tools

rustup target add thumbv7em-none-eabihf
```

For adding bootloader to Ubuntu: 
- Download Zip: https://github.com/PaulStoffregen/teensy_loader_cli/releases/tag/2.3
- Extract to home and `cd ~/teensy_loader_cli-2.3`, run `make`
- cp ~/teensy_loader_cli-2.3/teensy_loader_cli ~/.local/bin/teensy-loader-cli

`just upload` or `just u`

To install rust-analyzer: https://github.com/rust-lang/rust-analyzer/releases/download/2025-08-11/rust-analyzer-x86_64-unknown-linux-gnu.gz
Extract and: 
```
cp ~/rust-analyzer-x86_64-unknown-linux-gnu ~/.local/bin/rust-analyzer
chmod +x ~/.local/bin/rust-analyzer

```

Edit rust-analyzer extension settings and add .local/bin path 

