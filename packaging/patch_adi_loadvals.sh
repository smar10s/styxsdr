#!/bin/sh
# SPDX-License-Identifier: MIT
# packaging/patch_adi_loadvals.sh — On-device script to patch adi_loadvals_pluto
#
# Runs ON the PlutoSDR (busybox ash). Uploaded and executed by
# bin/fix_ad9361_guardrail.sh.
#
# Removes the guardrail that forces attr_val=ad9361 → ad9363a on non-Rev.C.
# All other adi_loadvals_pluto logic is preserved verbatim.

# The patched script value for fw_setenv.
# This is the v0.39 adi_loadvals_pluto with one block removed:
#   "if test "${attr_val}" = "ad9361" && test ! "${model}" = "..." ; then
#       setenv attr_val ad9363a; saveenv;
#   fi;"
#
# fw_setenv on busybox takes: fw_setenv <name> <value...>
# where <value...> is all remaining args joined by spaces.
# No quoting of the value is needed — it's literal from arg[2] onward.

fw_setenv adi_loadvals_pluto 'if test -n "${ad936x_ext_refclk}" && test ! -n "${ad936x_skip_ext_refclk}"; then fdt set /clocks/clock@0 clock-frequency ${ad936x_ext_refclk}; fi; if test -n "${ad936x_ext_refclk_override}"; then fdt set /clocks/clock@0 clock-frequency ${ad936x_ext_refclk_override}; fi; if test -n "${refclk_source}" && test ! "${refclk_source}" = "internal" && test ! "${refclk_source}" = "external"; then setenv refclk_source internal; saveenv; fi; if test "${refclk_source}" = "internal" && test "${model}" = "Analog Devices PlutoSDR Rev.C (Z7010/AD9363)" ; then fdt rm /amba/gpio@e000a000/clock_extern_en || fdt rm /axi/gpio@e000a000/clock_extern_en; fi; if test "${refclk_source}" = "external" && test "${model}" = "Analog Devices PlutoSDR Rev.C (Z7010/AD9363)" ; then fdt rm /amba/gpio@e000a000/clock_internal_en || fdt rm /axi/gpio@e000a000/clock_internal_en; fi; if test -n "${attr_val}" && test ! "${attr_val}" = "ad9361" && test ! "${attr_val}" = "ad9363a" && test ! "${attr_val}" = "ad9364"; then setenv attr_val ad9363a; saveenv; fi; if test -n "${mode}" && test ! "${mode}" = "1r1t" && test ! "${mode}" = "2r2t"; then setenv mode 1r1t; saveenv; fi; if test -n "${attr_name}" && test -n "${attr_val}"; then fdt set /amba/spi@e0006000/ad9361-phy@0 ${attr_name} ${attr_val} || fdt set /axi/spi@e0006000/ad9361-phy@0 ${attr_name} ${attr_val}; fi; if test "${mode}" = "1r1t" && test "${model}" = "Analog Devices PlutoSDR Rev.C (Z7010/AD9363)"; then fdt rm /amba/spi@e0006000/ad9361-phy@0 adi,2rx-2tx-mode-enable || fdt rm /axi/spi@e0006000/ad9361-phy@0 adi,2rx-2tx-mode-enable; fdt set /fpga-axi/cf-ad9361-dds-core-lpc@79024000 compatible adi,axi-ad9364-dds-6.00.a; fi; if test -n "${cs_gpio}" && test "${model}" = "Analog Devices PlutoSDR Rev.C (Z7010/AD9363)"; then fdt set /amba/axi_quad_spi@7C430000/ cs-gpios "<0x06 ${cs_gpio} 0>" || fdt set /axi/axi_quad_spi@7C430000/ cs-gpios "<0x06 ${cs_gpio} 0>"; fi; if test -n "${attr_val}" && test "${attr_val}" = "ad9364"; then fdt set /fpga-axi/cf-ad9361-dds-core-lpc@79024000 compatible adi,axi-ad9364-dds-6.00.a; if test ! "${mode}" = "1r1t"; then fdt rm /amba/spi@e0006000/ad9361-phy@0 adi,2rx-2tx-mode-enable || fdt rm /axi/spi@e0006000/ad9361-phy@0 adi,2rx-2tx-mode-enable; setenv mode 1r1t; saveenv; fi; fi;'

echo "adi_loadvals_pluto patched (ad9361 guardrail removed)"
