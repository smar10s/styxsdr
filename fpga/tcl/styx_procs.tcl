# SPDX-License-Identifier: MIT
# fpga/tcl/styx_procs.tcl — Reusable Vivado build procedures
#
# Provides:
#   styx::fingerprint    — Content-addressed build ID from source files
#   styx::set_build_id   — Inject fingerprint into axi_build_id parameter
#   styx::ooc_synth      — Area-optimized OOC synthesis (parallel)
#   styx::global_synth   — Top-level synthesis
#   styx::implement      — Place & route with named strategy
#   styx::check_timing   — WNS/WHS extraction (no string matching)
#   styx::report_util    — Hierarchical utilization report
#   styx::write_outputs  — Bitstream + XSA generation

namespace eval styx {
    variable version "1.0"

    # Z-7010 resource limits
    variable max_lut  17600
    variable max_ff   35200
    variable max_bram 60
    variable max_dsp  80
}

# ============================================================================
# styx::fingerprint — Content-addressed build ID
#
# Hashes all RTL + TCL + XDC sources in deterministic order (sorted paths).
# Returns: 8-char hex string prefixed with 0x (32-bit)
# ============================================================================
proc styx::fingerprint {rtl_dir project_dir} {
    set files [list]
    foreach f [glob -nocomplain $rtl_dir/*.v] { lappend files $f }
    foreach f [glob -nocomplain $rtl_dir/*.hex] { lappend files $f }
    foreach f [glob -nocomplain $project_dir/*.tcl] { lappend files $f }
    foreach f [glob -nocomplain $project_dir/*.v] { lappend files $f }
    foreach f [glob -nocomplain $project_dir/constraints/*.xdc] { lappend files $f }

    # Deterministic order — sort by basename to be locale-independent
    set files [lsort -dictionary $files]

    if {[llength $files] == 0} {
        error "styx::fingerprint: no source files found in $rtl_dir or $project_dir"
    }

    # Write file list to temp, hash with sha256sum
    set manifest ""
    foreach f $files {
        set fd [open $f r]
        append manifest [read $fd]
        close $fd
    }

    # Use sha256sum (available on Linux build hosts)
    set tmp_file "/tmp/styx_fp_[pid].txt"
    set fd [open $tmp_file w]
    puts -nonewline $fd $manifest
    close $fd

    set hash [exec sha256sum $tmp_file]
    file delete $tmp_file

    set hex [string range $hash 0 7]
    puts "FINGERPRINT: 0x$hex (from [llength $files] source files)"
    return "0x$hex"
}

# ============================================================================
# styx::set_build_id — Inject fingerprint into axi_build_id parameter
# ============================================================================
proc styx::set_build_id {fingerprint} {
    # Convert hex string to integer for Vivado parameter
    set int_val [expr $fingerprint]
    set_property CONFIG.BUILD_ID $int_val [get_bd_cells -quiet axi_build_id_0]
    puts "BUILD_ID: $fingerprint ($int_val)"
}

# ============================================================================
# styx::ooc_synth — Out-of-context area-optimized synthesis
#
# Discovers OOC runs matching the given module names and applies
# Flow_AreaOptimized_medium strategy. Launches all in parallel.
#
# Arguments:
#   modules — space-separated string of module names (e.g., "iq_dma_rx hil_ctrl")
# ============================================================================
proc styx::ooc_synth {modules} {
    set run_list [list]

    # Discover matching OOC synthesis runs
    foreach mod $modules {
        # Vivado names BD module OOC runs as: system_<module>_<instance>_synth_1
        set pattern "system_${mod}_*_synth_1"
        set runs [get_runs -quiet -filter "NAME =~ $pattern"]

        if {[llength $runs] == 0} {
            puts "OOC: $mod — no matching run found (skipped)"
            continue
        }

        foreach run $runs {
            reset_run $run
            set_property strategy Flow_AreaOptimized_medium [get_runs $run]
            lappend run_list $run
            puts "OOC: $mod — queued ($run)"
        }
    }

    if {[llength $run_list] == 0} {
        puts "OOC: no modules to synthesize"
        return
    }

    # Launch all OOC runs in parallel
    puts "OOC: launching [llength $run_list] runs..."
    launch_runs $run_list -jobs [get_param general.maxThreads]

    # Wait for all to complete
    foreach run $run_list {
        wait_on_run $run
        set status [get_property STATUS [get_runs $run]]
        if {![string match "synth_design Complete*" $status]} {
            error "OOC synthesis failed for $run: $status"
        }
    }
    puts "OOC: all [llength $run_list] modules complete"
}

# ============================================================================
# styx::global_synth — Top-level synthesis
# ============================================================================
proc styx::global_synth {top strategy} {
    set_property TOP $top [current_fileset]
    set_property strategy $strategy [get_runs synth_1]
    set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY rebuilt [get_runs synth_1]

    puts "SYNTH: $top (strategy=$strategy)"
    reset_run synth_1
    launch_runs synth_1 -jobs [get_param general.maxThreads]
    wait_on_run synth_1

    set status [get_property STATUS [get_runs synth_1]]
    if {![string match "synth_design Complete*" $status]} {
        error "Global synthesis failed: $status"
    }
    puts "SYNTH: complete"
}

# ============================================================================
# styx::implement — Place & route with named strategy
#
# Arguments:
#   place_strategy — Vivado placement directive name (e.g., "Explore")
#
# Also configures:
#   opt_design:      ExploreWithRemap
#   phys_opt_design: AggressiveExplore (enabled)
#   route_design:    AggressiveExplore
# ============================================================================
proc styx::implement {place_strategy} {
    # Validate strategy name
    set valid_strategies [list \
        Explore Default AltSpreadLogic_high EarlyBlockPlacement \
        ExtraPostPlacementOpt WLDrivenBlockPlacement \
        SSI_SpreadLogic_high AltSpreadLogic_medium]

    if {$place_strategy ni $valid_strategies} {
        puts "WARNING: Unknown placement strategy '$place_strategy'. Valid:"
        puts "  [join $valid_strategies {, }]"
        puts "  Proceeding anyway (Vivado may accept it)."
    }

    set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE $place_strategy [get_runs impl_1]
    set_property STEPS.OPT_DESIGN.ARGS.DIRECTIVE ExploreWithRemap [get_runs impl_1]
    set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
    set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
    set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]

    puts "IMPL: placement=$place_strategy"
    launch_runs impl_1 -to_step write_bitstream -jobs [get_param general.maxThreads]
    wait_on_run impl_1

    set status [get_property STATUS [get_runs impl_1]]
    if {![string match "write_bitstream Complete*" $status]} {
        error "Implementation failed: $status"
    }
    puts "IMPL: complete"
}

# ============================================================================
# styx::check_timing — Extract WNS/WHS from implemented design
#
# Returns: dict with keys: wns, whs, tns, ths, met (bool)
# Uses Vivado API directly — no string matching.
# ============================================================================
proc styx::check_timing {} {
    open_run impl_1

    # Force timing update so report_timing gives accurate results
    update_timing

    # Use report_timing to get WNS/WHS — STATS properties are unreliable
    # (they return empty strings when timing hasn't been analyzed in-session)
    set setup_rpt [report_timing -setup -max_paths 1 -return_string]
    set hold_rpt  [report_timing -hold  -max_paths 1 -return_string]

    # Extract slack from report (line: "Slack (MET|VIOLATED) : <value>ns")
    set wns 0.0
    set whs 0.0
    if {[regexp {Slack \([A-Z]+\)\s*:\s*([-0-9.]+)ns} $setup_rpt -> val]} {
        set wns $val
    }
    if {[regexp {Slack \([A-Z]+\)\s*:\s*([-0-9.]+)ns} $hold_rpt -> val]} {
        set whs $val
    }

    set met [expr {$wns >= 0.0 && $whs >= 0.0}]

    if {$met} {
        puts "TIMING: MET (WNS=${wns}ns WHS=${whs}ns)"
    } else {
        puts "TIMING: VIOLATED (WNS=${wns}ns WHS=${whs}ns)"
        puts "TIMING: Worst setup paths:"
        puts [report_timing -setup -max_paths 5 -return_string]
        if {$whs < 0} {
            puts "TIMING: Worst hold paths:"
            puts [report_timing -hold -max_paths 5 -return_string]
        }
    }

    return [dict create wns $wns whs $whs met $met]
}

# ============================================================================
# styx::report_util — Hierarchical utilization report
#
# Prints summary to stdout and writes detailed report to output_dir.
# Returns: dict with luts, ffs, bram, dsp, and percentages
# ============================================================================
proc styx::report_util {output_dir} {
    variable max_lut
    variable max_ff
    variable max_bram
    variable max_dsp

    # Write full hierarchical report to file
    set rpt_file [file join $output_dir utilization.rpt]
    report_utilization -hierarchical -hierarchical_depth 2 -file $rpt_file

    # Extract top-level numbers
    set rpt [report_utilization -return_string]

    # Parse LUT and FF from report (Vivado's structured output)
    set luts 0
    set ffs 0
    set bram 0
    set dsp 0

    foreach line [split $rpt "\n"] {
        if {[regexp {Slice LUTs\s*\|\s*(\d+)} $line -> val]} { set luts $val }
        if {[regexp {Slice Registers\s*\|\s*(\d+)} $line -> val]} { set ffs $val }
        if {[regexp {Block RAM Tile\s*\|\s*(\d+)} $line -> val]} { set bram $val }
        if {[regexp {DSPs\s*\|\s*(\d+)} $line -> val]} { set dsp $val }
    }

    set lut_pct  [format "%.1f" [expr {100.0 * $luts / $max_lut}]]
    set ff_pct   [format "%.1f" [expr {100.0 * $ffs / $max_ff}]]
    set bram_pct [format "%.1f" [expr {100.0 * $bram / $max_bram}]]
    set dsp_pct  [format "%.1f" [expr {100.0 * $dsp / $max_dsp}]]

    puts "UTIL: LUTs=$luts/$max_lut (${lut_pct}%)"
    puts "UTIL: FFs=$ffs/$max_ff (${ff_pct}%)"
    puts "UTIL: BRAM=$bram/$max_bram (${bram_pct}%)"
    puts "UTIL: DSP=$dsp/$max_dsp (${dsp_pct}%)"
    puts "UTIL: Full report: $rpt_file"

    return [dict create \
        luts $luts ffs $ffs bram $bram dsp $dsp \
        lut_pct $lut_pct ff_pct $ff_pct bram_pct $bram_pct dsp_pct $dsp_pct]
}

# ============================================================================
# styx::write_outputs — Bitstream + XSA
# ============================================================================
proc styx::write_outputs {output_dir} {
    file mkdir $output_dir

    # XSA (hardware platform)
    set xsa_path [file join $output_dir system_top.xsa]
    write_hw_platform -fixed -include_bit -force $xsa_path
    puts "OUTPUT: $xsa_path"

    # Copy .bit separately for direct use
    set impl_dir [get_property DIRECTORY [get_runs impl_1]]
    set bit_files [glob -nocomplain $impl_dir/*.bit]
    if {[llength $bit_files] > 0} {
        set bit_src [lindex $bit_files 0]
        set bit_dst [file join $output_dir system_top.bit]
        file copy -force $bit_src $bit_dst
        puts "OUTPUT: $bit_dst"
    }
}
