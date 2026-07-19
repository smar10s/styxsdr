# SPDX-License-Identifier: MIT
# fpga/project/system_project.tcl — Vivado project creation for Styx
#
# Creates the Vivado project with all sources and block design.
# Does NOT configure synthesis/implementation strategies or run builds —
# that's build.tcl's job.
#
# Called by: build.tcl (once, or when BD/sources change)
# Expects: $ad_hdl_dir set before sourcing

# If ad_hdl_dir not set, derive from directory structure
if {![info exists ad_hdl_dir] || $ad_hdl_dir eq ""} {
    set ad_hdl_dir [file normalize [file dirname [info script]]/../extern/adi-hdl]
}

source $ad_hdl_dir/projects/scripts/adi_project_xilinx.tcl
source $ad_hdl_dir/projects/scripts/adi_board.tcl

adi_project_create pluto 0 {} "xc7z010clg225-1"

# Add RTL sources from fpga/rtl/
set rtl_dir [file normalize [file dirname [info script]]/../rtl]
set project_dir [file dirname [info script]]

adi_project_files pluto [list \
    "$project_dir/system_top.v" \
    "$project_dir/constraints/system_constr.xdc" \
    "$ad_hdl_dir/library/common/ad_iobuf.v" \
]

# Add all custom RTL
foreach f [lsort [glob $rtl_dir/*.v]] {
    add_files -norecurse -fileset sources_1 $f
}

# Disable auto-generated PS7 constraints (we use our own)
set_property is_enabled false [get_files *system_sys_ps7_0.xdc]

# Use all available CPUs
set_param general.maxThreads [exec nproc]

# Run block design generation (system_bd.tcl) via ADI framework
# ADI_SKIP_SYNTHESIS must be set to prevent adi_project_run from launching synth
set ::env(ADI_SKIP_SYNTHESIS) 1
set ::env(ADI_IGNORE_VERSION_CHECK) 1

adi_project_run pluto
