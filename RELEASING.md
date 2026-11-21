# Releasing StyxSDR

## Prerequisites

- Vivado 2025.2 (local or via remote build host configured in `config.mk`)
- Docker
- Write access to the repository

## Steps

1. **Bump the version**

   Set `version` in `pyproject.toml` to the release version.

2. **Full build**

   ```bash
   make all    # bitstream + firmware + package
   ```

   Or step-by-step:

   ```bash
   make bitstream
   make firmware
   make package
   ```

3. **Verify the artifact**

   The output is `build/fpga/pluto.frm`.  Confirm the fingerprint:

   ```bash
   cat build/fpga/fingerprint         # e.g. 0x69962f47
   cat build/fpga/timing_status       # should be "met"
   ```

4. **Tag and push**

   ```bash
   VERSION=$(grep version pyproject.toml | head -1 | sed 's/.*"\(.*\)".*/\1/')
   git tag -a "v$VERSION" -m "v$VERSION"
   git push origin "v$VERSION"
   ```

5. **Create GitHub Release**

   - Title: `vX.Y.Z` (match the tag)
   - Attach `build/fpga/pluto.frm`
   - Copy the release notes template below, filling in the fingerprint

### Release Notes Template

```
StyxSDR vX.Y.Z — PlutoSDR firmware image

- Build fingerprint: 0x69962f47
- Timing: met
- Vivado: 2025.2

Flash via USB mass storage: copy pluto.frm to the Pluto drive, eject, and wait.
See README for full instructions.
```
