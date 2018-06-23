# HelenOS-AArch64

HelenOS-AArch64 is a project that adds support for the 64-bit ARM architecture
(AArch64) to [HelenOS][HelenOS].

The port was primarily created for learning purposes by its author. The system
is able to boot under [QEMU][QEMU] and allows to execute user commands entered
on the console, but otherwise provides limited functionality and has not been
extensively tested.

To build and run this port, follow [the official build instructions for
HelenOS][CompilingFromSource] with these deviations:

1. Get the sources

        $ git clone https://github.com/setupji/helenos.git

2. Build a supported cross-compiler

        $ cd helenos-aarch64/tools
        $ ./toolchain.sh arm64

3. Did you install the compiler toolchain? Good.

4. Configure and build

        $ cd ..
        $ make PROFILE=arm64/virt

5. Run it

        $ ./tools/ew.py

Bug reports and any questions about the port can be sent to <setup@dagobah.cz>.

[HelenOS]: http://www.helenos.org/
[QEMU]: https://www.qemu.org/
[CompilingFromSource]: http://www.helenos.org/wiki/UsersGuide/CompilingFromSource
