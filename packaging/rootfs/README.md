# Rootfs branding

These files replace the stock ADI PlutoSDR branding in the rootfs.

- `motd` — displayed after SSH login (Message of the Day)
- `issue` — displayed before login prompt

To repack: extract `rootfs.cpio.gz`, copy these into `etc/`, then `cpio | gzip`.

The `opt/VERSIONS` file is generated dynamically and should not be committed.
