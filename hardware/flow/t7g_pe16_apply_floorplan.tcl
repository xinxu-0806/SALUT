# T7g (v661 PE16, SLR0-relief partition; copied from v659): T7f rules (absent constant write-side META skids allowed) on the PE16 owner data.
# the META m_axi interfaces are read-only, so synthesis removes those constant stages. Any other missing cell is an error.
# Hard full-SLR pblocks under pblock_dynamic_region (same geometry rule as the v636 T7 script).
# Owners come from the data-only T7c_owners_DATA_NOT_native.tcl. No clock / DRC / exception changes.
namespace eval ::v659fp {
  variable dir [file dirname [file normalize [info script]]]
  proc require {ok why} {if {!$ok} {error $why}}
  proc hier_cell {path} {
    set cell [get_cells -quiet $path]
    if {[llength $cell] != 1 || [get_property NAME $cell] ne $path || [get_property IS_PRIMITIVE $cell]} {return {}}
    return $cell
  }
  proc relay_child {root relay stage} {
    set want $root/$relay/$stage; set found {}
    foreach c [get_cells -quiet -filter {IS_PRIMITIVE == 0} $root/$relay/*] {
      if {[get_property NAME $c] eq $want} {lappend found $c}
    }
    return $found
  }
  proc apply {report} {
    variable dir
    set root level0_i/ulp/SparseLUKernel_1/inst
    source [file join $dir T7c_owners_DATA_NOT_native.tcl]
    source [file join $dir T7g_pe16_ctrl_owners_DATA_NOT_native.tcl]
    require [expr {[dict size $v659_ctrl] == 1581}] CTRL_OWNER_SCOPE
    require [expr {[dict size $v642_tasks] == 289 && [dict size $v642_local_fifos] == 602 &&
                   [dict size $v642_relay_stages] == 351 && [dict size $v642_axi_wrappers] == 24}] OWNER_DATA_SCOPE
    require [expr {[llength [get_pblocks -quiet salut_banked_slr*]] == 0}] NO_PREEXISTING_SALUT_PBLOCKS
    set plan {}
    foreach {name slr} $v642_tasks {
      set c [hier_cell $root/$name]; require [llength $c] "TASK_MISSING $name"; lappend plan task $name $c $slr
    }
    set optimized_local 0
    foreach {name slr} $v642_local_fifos {
      if {[llength [get_cells -quiet $root/$name]] == 0} {incr optimized_local; continue}
      set c [hier_cell $root/$name]; require [llength $c] "LOCAL_FIFO_AMBIGUOUS $name"; lappend plan local_fifo $name $c $slr
    }
    set optimized {}
    foreach {key slr} $v642_relay_stages {
      require [regexp {^([A-Za-z0-9_]+)/(inst\[[0-3]\]\.unit)$} $key -> relay stage] "RELAY_KEY $key"
      if {[llength [get_cells -quiet $root/$relay]] == 0} {
        require [expr {$relay in $v642_constant_relays}] "RELAY_MISSING_NOT_PROVEN_CONSTANT $relay"
        dict set optimized $relay 1; continue
      }
      set c [relay_child $root $relay $stage]
      require [expr {[llength $c] == 1}] "RELAY_STAGE_MISSING_OR_AMBIGUOUS $key [llength $c]"
      lappend plan relay_stage $key $c $slr
    }
    require [expr {[lsort [dict keys $optimized]] eq [lsort $v642_constant_relays]}] "OPTIMIZED_RELAY_SET_EXACT [lsort [dict keys $optimized]]"
    foreach {name slr} $v642_axi_wrappers {
      set c [hier_cell $root/$name]; require [llength $c] "AXI_WRAPPER_MISSING $name"; lappend plan axi_wrapper $name $c $slr
    }
    set absent_const_axi {}
    foreach {name slr} $v659_ctrl {
      set c [hier_cell $root/$name]
      if {[llength $c] == 0} {set c [get_cells -quiet -filter {IS_PRIMITIVE} $root/$name/*]}
      if {[llength $c] == 0 && [regexp {^(salut_rst_leaf_)?salut_axi_Parallel(FrontEmitter|Scheduler|SegmentEmitter)_0_(AW|W)_[pt]$} $name]} {
        lappend absent_const_axi $name; continue
      }
      require [expr {[llength $c] >= 1}] "CTRL_CELL_MISSING $name"; lappend plan ctrl $name $c $slr
    }
    foreach {name slr} $v642_control {
      set c [hier_cell $root/$name]; require [llength $c] "CONTROL_MISSING $name"; lappend plan control $name $c $slr
    }
    foreach slr {SLR0 SLR1 SLR2} {
      set physical [get_slrs -quiet $slr]; require [expr {[llength $physical] == 1}] "DEVICE_SLR $slr"
      set ranges {}
      foreach region [get_clock_regions -quiet -of_objects $physical] {
        set n [get_property NAME $region]; require [regexp {^X[0-9]+Y[0-9]+$} $n] "CLOCK_REGION $n"
        lappend ranges CLOCKREGION_${n}:CLOCKREGION_${n}
      }
      require [llength $ranges] "NO_CLOCK_REGIONS $slr"
      set pb [create_pblock salut_banked_[string tolower $slr]]
      set_property PARENT pblock_dynamic_region $pb
      resize_pblock $pb -add $ranges
      set_property IS_SOFT false $pb
      set_property CONTAIN_ROUTING false $pb
      set pblock($slr) $pb
    }
    set fd [open $report {WRONLY CREAT EXCL}]
    puts $fd "kind\tname\tslr"
    set counts {}
    foreach {kind name cell slr} $plan {
      set_property USER_SLR_ASSIGNMENT $slr $cell
      add_cells_to_pblock $pblock($slr) $cell
      puts $fd "$kind\t$name\t$slr"
      dict incr counts $kind
    }
    close $fd
    return [dict create counts $counts optimized_local_fifos $optimized_local optimized_constant_relays [lsort [dict keys $optimized]] absent_constant_write_side_meta_axi [lsort $absent_const_axi]]
  }
}
