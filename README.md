## Flashing / programming

Wire serial adaptor:

* black cable on `Ground`
* yellow cable on `A9`
* orange cable on `A10`

Set jumper to programming mode: `BOOT0: 1`, `BOOT1: 0`.

Flash using PlatformIO upload target.

## Normal use

Set jumper to normal boot mode: `BOOT0: 0` `BOOT1: 0`.
