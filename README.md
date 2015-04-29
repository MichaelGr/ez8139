Hello,

ez8139 is a simpler version of RTL8139 ethernet driver in Linux (in 8139cp.c file). I am writing it as an educational piece of code to learn device driver concepts. Things like PCI interface, DMA access, address spaces etc.

It will be simpler since it will only work in x86_64 for QEMU and code for extra stuff like WOL, eeprom programming etc. won't be included.

I am still learning device drivers and kernel in general so there will be errors on my side. Please contact me if you think something is wrong with the code.

Like other kernel code I share in GitHub, you can create a symbolic link to your kernel directory with name kerneldir and execute farmake.script and it will compile the module against your kernel. Please make sure that the actual Realtek 8139 driver is not compiled inside your kernel binary. I am compiling and testing under Gentoo in a x86_64 QEMU environment with Linux 3.14.22 kernel.

Under any circumstances this module should not be run on a production system.
