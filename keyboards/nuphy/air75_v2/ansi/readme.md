# NuPhy Air75 V2

*NuPhy Air75 V2 is a standard 84 key keyboard.*
![NuPhy Air75 V2](https://i.imgur.com/R7jS2JC.jpeg)

* Keyboard Maintainer: [nuphy](https://github.com/nuphy-src)
* Hardware Supported: NuPhy Air75 V2 PCB
* Hardware Availability: Private

Make example for this keyboard (after setting up your build environment):

    make nuphy/air75_v2/ansi:via

Flashing example for this keyboard:

    make nuphy/air75_v2/ansi:via:flash

See the [build environment setup](https://docs.qmk.fm/#/getting_started_build_tools) and the [make instructions](https://docs.qmk.fm/#/getting_started_make_guide) for more information. Brand new to QMK? Start with our [Complete Newbs Guide](https://docs.qmk.fm/#/newbs).

## Wireless report delivery

The UART receiver retains partial frames, validates their lengths and checksums,
and processes consecutive frames separately. Three-byte acknowledgements do not
update connection or battery data.

Reports buffered during reconnect are sent in order, one per task iteration,
without an additional 30 ms delay. If the 63-report queue fills, older states
are discarded only when a newer state of the same report type is available.
The latest state of every type, including key releases, is retained; some
intermediate keystrokes or mouse movements can still be lost on overflow.

Keyboard reports are transmitted three times using NuPhy's UART guard timing.
Mouse and media reports are transmitted once to avoid repeating actions.
Keyboard retries preserve both NKRO halves when other report types are sent.

## Bootloader

Enter the bootloader in one way:

* **Bootmagic reset**: Hold down the key at (0,0) in the matrix (usually the top left key or Escape) and plug in the keyboard

* **Hardware reset**: Remove the capslock keycap, hold the little button beneath and plug in the keyboard.
