# SPDX-License-Identifier: MIT
# fpga/tcl/build.tcl — Vivado batch build driver for Styx
#
# Usage:
#   vivado -mode batch -nojournal -nolog -source fpga/tcl/build.tcl -tclargs \
#       -project_dir fpga/project \
#       -rtl_dir fpga/rtl \
#       -adi_hdl_dir fpga/extern/adi-hdl \
#       -output_dir build/fpga \
#       [-place_strategy Explore] \
#       [-ooc_modules "mod1 mod2 ..."] \
#       [-incremental path/to/reference.dcp] \
#       [-skip_project]
#
# Phases:
#   1. Project creation (or open existing)
#   2. Build fingerprint injection
#   3. OOC synthesis (area-optimized, parallel)
#   4. Global synthesis
#   5. Implementation (placement + routing)
#   6. Timing check + utilization report
#   7. Write bitstream, XSA, DCP
#
# Exit codes:
#   0 = success (timing met)
#   1 = build error
#   2 = timing violation (bitstream still generated)

source [file join [file dirname [info script]] styx_procs.tcl]

# ============================================================================
# Argument parsing
# ============================================================================

proc parse_args {argv} {
    set opts [dict create \
        project_dir     "" \
        rtl_dir         "" \
        adi_hdl_dir     "" \
        output_dir      "" \
        place_strategy  "Explore" \
        ooc_modules     {} \
        incremental     "" \
        skip_project    0 \
    ]

    for {set i 0} {$i < [llength $argv]} {incr i} {
        set arg [lindex $argv $i]
        switch -- $arg {
            -project_dir    { dict set opts project_dir [lindex $argv [incr i]] }
            -rtl_dir        { dict set opts rtl_dir [lindex $argv [incr i]] }
            -adi_hdl_dir    { dict set opts adi_hdl_dir [lindex $argv [incr i]] }
            -output_dir     { dict set opts output_dir [lindex $argv [incr i]] }
            -place_strategy { dict set opts place_strategy [lindex $argv [incr i]] }
            -ooc_modules    { dict set opts ooc_modules [lindex $argv [incr i]] }
            -incremental    { dict set opts incremental [lindex $argv [incr i]] }
            -skip_project   { dict set opts skip_project 1 }
            default         { puts "WARNING: unknown argument: $arg" }
        }
    }

    foreach key {project_dir rtl_dir adi_hdl_dir output_dir} {
        if {[dict get $opts $key] eq ""} {
            error "Required argument -$key not provided"
        }
    }
    return $opts
}

set opts [parse_args $argv]

set project_dir    [file normalize [dict get $opts project_dir]]
set rtl_dir        [file normalize [dict get $opts rtl_dir]]
set adi_hdl_dir    [file normalize [dict get $opts adi_hdl_dir]]
set output_dir     [file normalize [dict get $opts output_dir]]
set place_strategy [dict get $opts place_strategy]
set ooc_modules    [dict get $opts ooc_modules]
set incremental    [dict get $opts incremental]

file mkdir $output_dir

# ============================================================================
# Phase 1: Project creation
# ============================================================================

if {![dict get $opts skip_project]} {
    puts "========== PHASE 1: PROJECT CREATION =========="

    # Set ADI env for their scripts (ad_hdl_dir is what ADI procs expect)
    set ad_hdl_dir $adi_hdl_dir

    # Skip ADI version check (we use Vivado 2025.2, ADI HDL expects 2023.2)
    set ::env(ADI_IGNORE_VERSION_CHECK) 1
    set ::env(ADI_SKIP_SYNTHESIS) 1

    # Source ADI env setup (sets IGNORE_VERSION_CHECK, required_vivado_version, etc.)
    source $adi_hdl_dir/scripts/adi_env.tcl

    cd $project_dir
    source $project_dir/system_project.tcl

    puts "Project created: $project_dir/pluto.xpr"
} else {
    puts "========== PHASE 1: OPENING EXISTING PROJECT =========="
    open_project $project_dir/pluto.xpr
}

# ============================================================================
# Phase 2: Build fingerprint
# ============================================================================

puts "========== PHASE 2: BUILD FINGERPRINT =========="
set fingerprint [styx::fingerprint $rtl_dir $project_dir]
styx::set_build_id $fingerprint

# ============================================================================
# Phase 3: OOC synthesis
# ============================================================================

if {[llength $ooc_modules] > 0} {
    puts "========== PHASE 3: OOC SYNTHESIS ([llength $ooc_modules] modules) =========="
    styx::ooc_synth $ooc_modules
} else {
    puts "========== PHASE 3: OOC SYNTHESIS (skipped — no modules specified) =========="
}

# ============================================================================
# Phase 4: Global synthesis
# ============================================================================

puts "========== PHASE 4: GLOBAL SYNTHESIS =========="
styx::global_synth system_top Flow_AreaOptimized_high

# ============================================================================
# Phase 5: Implementation
# ============================================================================

puts "========== PHASE 5: IMPLEMENTATION (strategy=$place_strategy) =========="

# Incremental implementation: if a reference DCP exists, use it
if {$incremental ne "" && [file exists $incremental]} {
    puts "INCREMENTAL: Using reference checkpoint: $incremental"
    set_property INCREMENTAL_CHECKPOINT $incremental [get_runs impl_1]
}

styx::implement $place_strategy

# ============================================================================
# Phase 6: Reports
# ============================================================================

puts "========== PHASE 6: TIMING + UTILIZATION =========="

set timing [styx::check_timing]
styx::report_util $output_dir

# ============================================================================
# Phase 7: Write outputs
# ============================================================================

puts "========== PHASE 7: WRITE OUTPUTS =========="
styx::write_outputs $output_dir

# Save DCP for incremental builds
set dcp_path [file join $output_dir styx_impl.dcp]
write_checkpoint -force $dcp_path
puts "DCP: $dcp_path (use with -incremental for faster rebuilds)"

# Write fingerprint file
set fp_fd [open [file join $output_dir fingerprint] w]
puts $fp_fd $fingerprint
close $fp_fd

# ============================================================================
# Done
# ============================================================================

if {[dict get $timing met]} {
    puts "\n========== BUILD SUCCESS =========="
    puts "  Fingerprint: $fingerprint"
    puts "  Timing:      MET (WNS=[dict get $timing wns]ns)"
    puts "  Output:      $output_dir"
    exit 0
} else {
    puts "\n========== BUILD COMPLETE (TIMING VIOLATED) =========="
    puts "  Fingerprint: $fingerprint"
    puts "  WNS:         [dict get $timing wns]ns"
    puts "  Strategy:    $place_strategy"
    puts "  Output:      $output_dir (bitstream generated despite violation)"
    puts ""
    puts "  Try: make bitstream PLACE_STRATEGY=EarlyBlockPlacement"
    puts "  Try: make bitstream PLACE_STRATEGY=AltSpreadLogic_high"
    exit 2
}
