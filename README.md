# Embedded-Rust-Demo

Dies ist der Code für meine Demo im Video "Einführung zu Rust für Embedded Devices".
Getestet wurde es lediglich auf einem STM32F4 Board, mit einem LSM9DS1 IMU.

## Zum starten:
Rust installieren: https://www.rust-lang.org/learn/get-started

Die Flash-Utility installieren:
`cargo install cargo-flash`
`cargo install probe-run`

Den Linker installieren:
`cargo install flip-link`

Den Code Compilieren und auf das über USB verbundene Board flashen. Im Code-Verzeichniss:
`cargo run --release`

## Weiterlesen

der verwendete HAL: https://github.com/David-OConnor/stm32-hal
https://github.com/David-OConnor/stm32-hal-quickstart

RTIC: https://rtic.rs/0.5/book/en/preface.html
