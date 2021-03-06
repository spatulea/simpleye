## openocd
[User guide](https://openocd.org/doc-release/html/index.html)

Start openocd GDB server targetting nrf52 with stlink:
```zsh
openocd -f interface/stlink.cfg -f target/nrf52.cfg
```
Optionally may need these two options:
```zsh
openocd -f interface/stlink.cfg -f target/nrf52.cfg -c "gdb_flash_program enable" -c "gdb_breakpoint_override hard"
```
To flash hex file:
```zsh
openocd -f interface/stlink.cfg -f target/nrf52.cfg -c "program build/nrf52840_xxaa.hex verify reset"
```
```
Command: gdb_breakpoint_override [hard|soft|disable]
Force breakpoint type for gdb break commands. This option supports GDB GUIs which don’t distinguish hard versus soft breakpoints, if the default OpenOCD and GDB behaviour is not sufficient. GDB normally uses hardware breakpoints if the memory map has been set up for flash regions.
```
```
Config Command: gdb_flash_program (enable|disable)
Set to enable to cause OpenOCD to program the flash memory when a vFlash packet is received. The default behaviour is enable.
```
Nordic nRF52840 [memory map](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v15.3.0%2Flib_bootloader.html&cp=5_0_3_5_0_7&anchor=lib_bootloader_memory)