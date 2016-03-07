HelenOS-AArch64 is a project that adds support for the 64-bit ARM architecture
(AArch64) to [HelenOS][HelenOS].

To build and run this port, follow [official HelenOS build
instructions][CompilingFromSource] with these deviations:
1. Get the sources
   ```sh
   $ git clone git@bitbucket.org:setupji/helenos-aarch64.git
   ```
2. Build a supported cross-compiler
   ```sh
   $ cd helenos-aarch64/tools
   $ ./toolchain.sh arm64
   ```
3. Did you install the compiler toolchain? Good.
4. Configure and build
   ```sh
   $ cd ..
   $ make PROFILE=arm64/virt
   ```
5. Run it
   ```sh
   $ ./tools/ew.py
   ```

Please send bug reports and any questions about the project to:
Petr Pavlu <setup@dagobah.cz>

[HelenOS]: http://www.helenos.org/
[CompilingFromSource]: http://www.helenos.org/wiki/UsersGuide/CompilingFromSource
