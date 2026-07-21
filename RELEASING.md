# Releasing StyxSDR

## Prerequisites

- Vivado 2025.2 (local or via remote build host configured in `config.mk`)
- Docker
- Write access to the repository

## Steps

1. **Full build**

   ```bash
   make all    # bitstream + firmware + package
   ```

   Or step-by-step:

   ```bash
   make bitstream
   make firmware
   make package
   ```

2. **Verify the artifact**

   The output is `build/fpga/pluto.frm`.  Confirm the fingerprint:

   ```bash
   cat build/fpga/fingerprint         # e.g. 0x69962f47
   cat build/fpga/timing_status       # should be "met"
   ```

3. **Tag and push**

   ```bash
   git tag v$(grep version pyproject.toml | head -1 | sed 's/.*"\(.*\)".*/\1/')
   git push origin --tags
   ```

4. **Create GitHub Release**

   - Title: `v0.1.0` (match the tag)
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
