# v694: pre-placement child pblocks (inside the platform's own pblock_dynamic_SLR0/1) for the 1,282 hmss path_27
# SLR0<->SLR1 hop registers, next to the only usable RP SLL column (LAGUNA tile X104) and inside the Laguna row bands
# (SLR0 Y180-239, SLR1 Y240-299). Rectangles are clipped to the parent's DERIVED SLICE ranges. Placement-only constraint.
namespace eval ::v694 {
  proc clip {parent x0 y0 x1 y1} {
    set out {}
    foreach g [get_property DERIVED_RANGES [get_pblocks $parent]] {
      if {![regexp {^SLICE_X([0-9]+)Y([0-9]+):SLICE_X([0-9]+)Y([0-9]+)$} $g -> a b c d]} continue
      set ix0 [expr {max($a,$x0)}]; set iy0 [expr {max($b,$y0)}]; set ix1 [expr {min($c,$x1)}]; set iy1 [expr {min($d,$y1)}]
      if {$ix0 <= $ix1 && $iy0 <= $iy1} { lappend out "SLICE_X${ix0}Y${iy0}:SLICE_X${ix1}Y${iy1}" }
    }
    return $out
  }
  proc apply {listfile report} {
    set f [open $listfile]; set rows [split [string trim [read $f]] "\n"]; close $f
    set cells(0) {}; set cells(1) {}
    foreach r $rows { lassign [split $r "\t"] n s; set c [get_cells -quiet $n]; if {[llength $c] != 1} {error "HOP_CELL_MISSING $n"}; lappend cells($s) $c }
    set rep {}
    foreach s {0 1} y0 {180 240} y1 {239 299} {
      set parent pblock_dynamic_SLR$s
      set rg [clip $parent 145 $y0 177 $y1]
      if {[llength $rg] == 0} {error "NO_RANGE $s"}
      set pb [create_pblock v694_hmss27_hop_slr$s]
      set_property PARENT $parent $pb
      resize_pblock $pb -add $rg
      set_property IS_SOFT false $pb
      add_cells_to_pblock $pb $cells($s)
      lappend rep "slr$s cells=[llength $cells($s)] ranges=$rg"
    }
    set f [open $report w]; puts $f [join $rep "\n"]; close $f
    return $rep
  }
}
