# Double DAQ

RP-Pico programs that aggregate highly transient data via SPI onto a single SD card.

`central.c` and `peripheral.c` define the main capture loop.

SD functions (and flash functions, which are not used) are mostly taken from the old `pio_logic_analyzer` repo, but `config.txt` is different (prefixes are labeled now) and `meta.txt` doesn't contain as much info.

Writeup coming soon.
