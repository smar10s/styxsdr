# docker/sysroot

This directory holds ARM shared libraries extracted from the ADALM-Pluto
for cross-linking. The files are not checked in to the repository.

## Required files

```
usr/lib/libiio.so.0.25   # ARM shared lib from device
usr/lib/libiio.so.0      # symlink -> libiio.so.0.25
usr/lib/libiio.so        # symlink -> libiio.so.0
```

## How to populate

**Automatic** (Pluto connected via USB):

```bash
./scripts/hardware/build_arm.sh
```

The build script detects missing files and extracts them from the device
at `192.168.2.1` (default Pluto IP, password: `analog`).

**Manual:**

```bash
mkdir -p docker/sysroot/usr/lib
sshpass -p analog scp -O root@192.168.2.1:/usr/lib/libiio.so.0.25 \
    docker/sysroot/usr/lib/libiio.so.0.25
cd docker/sysroot/usr/lib
ln -sf libiio.so.0.25 libiio.so.0
ln -sf libiio.so.0 libiio.so
```

## Why not vendored

libiio is LGPL-licensed. Redistributing the binary in a source repo creates
license compliance obligations. Extracting directly from the target device
ensures version-matched linking without redistribution concerns.
