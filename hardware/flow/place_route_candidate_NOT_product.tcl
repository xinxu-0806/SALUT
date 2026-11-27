# v694 (= v662 script + path_27 hop child pblocks after the floorplan): place + AlternateCLBRouting route of the v661 PE16 (SLR0-relief) candidate_pe16_g (routability check, NOT product) (v645 + reset tree + scalar/ap_start/META-AXI pipelines + top keep_hierarchy) normal-SDK post-opt checkpoint with the T7c floorplan.
# Usage: vivado -mode batch -source this.tcl -tclargs <out_subdir> <place_directive>
# No clock / DRC severity / timing exception / netlist changes. Route and SDK are NOT queued.
namespace eval ::place637 {
  variable here [file dirname [file normalize [info script]]]
  variable out {};variable placecode NOT_ATTEMPTED;variable saved {}
  proc require {ok why} {if {!$ok} {error $why}}
  proc sha {path} {lindex [exec sha256sum -- $path] 0}
  proc save {name data} {
    variable out
    set fd [open $out/$name {WRONLY CREAT EXCL}]
    puts $fd [list [dict merge [dict create origin vivado native_pid [pid] local_time [clock format [clock seconds] -format {%Y-%m-%dT%H:%M:%S%z}]] $data [dict create accepted false]]]
    close $fd
  }
  proc command {tag words} {
    save ${tag}_BEGIN_NOT_product.tcllist [dict create command $words]
    set t0 [clock seconds]
    set code [catch {uplevel #0 $words} result options]
    save ${tag}_RESULT_NOT_product.tcllist [dict create actual_command_code $code actual_result $result elapsed_s [expr {[clock seconds]-$t0}]]
    if {$code} {return -options $options $result};return $result
  }
  proc report {tag words} {catch {command $tag $words}}
  proc constraints {} {
    set clocks {};foreach c [lsort [get_clocks *]] {dict set clocks [get_property NAME $c] [get_property PERIOD $c]}
    return [dict create clocks $clocks HDPR5 [get_property SEVERITY [get_drc_checks HDPR-5]]]
  }
  proc run {subdir directive} {
    variable here;variable out;variable placecode;variable saved
    require [regexp {^[A-Za-z0-9_]+$} $subdir] SUBDIR
    require [expr {$directive in {SSI_SpreadSLLs SSI_BalanceSLLs Explore Default AltSpreadLogic_high SSI_SpreadLogic_high SSI_HighUtilSLRs}}] DIRECTIVE
    set out $here/$subdir;file mkdir $out
    set cp build/fpga/banked-lu-dispatch-bram.kpvu1VPp/physical_audits/v719-LowFreq150Capture.lZV4hj5M/candidate_pe16_g_150/quick_pnr/cache/75b5df2e009dd736b88ac36f75aead92b898e0f239774c85d6fc259d8f4a11a9/postopt.dcp
    set cp_sha d027e381836c5eb3f0bc42253348d5a65ca1d912aa98374de1a8d0ae5150158f
    set apply build/fpga/banked-lu-dispatch-bram.kpvu1VPp/physical_audits/v661-PE16SLR0Relief.8o5DmUfC/t7g_pe16_apply_floorplan.tcl
    set owners build/fpga/banked-lu-dispatch-bram.kpvu1VPp/physical_audits/v661-PE16SLR0Relief.8o5DmUfC/T7g_pe16_ctrl_owners_DATA_NOT_native.tcl
    require [expr {[sha $cp] eq $cp_sha}] EXACT_CANDIDATE_B_POSTOPT
    save BEGIN_NOT_product.tcllist [dict create input $cp input_sha256 $cp_sha script_sha256 [sha [file normalize [info script]]] apply_sha256 [sha $apply] owners_sha256 [sha $owners] directive $directive xo_sha256 9d7f9a5c247bb4c32a02caa3340b5426fd7c07c3cd5e6a7dd3927b75a933ce3b]
    command OPEN [list open_checkpoint $cp]
    source scripts/assert_full_platform.tcl
    salut_assert_full_platform
    set bounds [constraints]
    set n [llength [get_cells -hierarchical -filter {IS_PRIMITIVE}]]
    save INPUT_SCOPE_NOT_product.tcllist [dict create primitives $n clock_count [dict size [dict get $bounds clocks]] constraints $bounds]
    require [expr {[expr {abs([get_property PERIOD [get_clocks clk_kernel_00_unbuffered_net]]-6.667)<0.002}] && [dict get $bounds HDPR5] eq "Error"}] USER_AUTHORISED_150MHZ_AND_DEFAULT_HDPR5
    source $apply
    set applied [::v659fp::apply $out/relay_floorplan_assignments.tsv]
    source $here/v694_path27_hop_pblocks.tcl
    save HOP_PBLOCKS_NOT_product.tcllist [dict create result [::v694::apply $here/path27_hop_regs.tsv $out/hop_pblocks.txt] list_sha256 [sha $here/path27_hop_regs.tsv] tcl_sha256 [sha $here/v694_path27_hop_pblocks.tcl]]
    save FLOORPLAN_APPLIED_NOT_product.tcllist $applied
    require [expr {[constraints] eq $bounds}] CLOCKS_AND_DRC_UNCHANGED_BY_FLOORPLAN
    set_param logicopt.enableBUFGinsertHFN 0
    set placecode [catch {command PLACE [list place_design -directive $directive -no_bufg_opt -ultrathreads]} result options]
    if {$placecode} {return -options $options $result}
    require [expr {[constraints] eq $bounds}] CLOCKS_AND_DRC_UNCHANGED_BY_PLACE
    set newcp $out/salut_relay_PLACED_${directive}_NOT_product.dcp
    command SAVE_PLACED [list write_checkpoint $newcp]
    set saved $newcp
    save PLACED_ARCHIVE_NOT_product.tcllist [dict create checkpoint $newcp candidate_sha256 [sha $newcp] primitives_after [llength [get_cells -hierarchical -filter {IS_PRIMITIVE}]] NOT_route_STA_PR_or_product true]
    report SLR_UTIL [list report_utilization -slr -file $out/placed_SLR_utilization.rpt]
    report CONGESTION [list report_design_analysis -congestion -min_congestion_level 3 -file $out/placed_congestion.rpt]
    report TIMING_SUMMARY [list report_timing_summary -delay_type min_max -report_unconstrained -max_paths 20 -file $out/placed_allclock_timing_summary.rpt]
    report KERNEL_TOP200 [list report_timing -group [get_clocks clk_kernel_00_unbuffered_net] -max_paths 200 -nworst 1 -unique_pins -file $out/placed_kernel_top200_paths.rpt]
    report QOR [list report_qor_assessment -file $out/placed_qor_assessment.rpt]
    report DRC [list report_drc -file $out/placed_default_DRC.rpt]
    set errors {};foreach v [get_drc_violations] {if {[get_property SEVERITY $v] eq "Error"} {lappend errors [get_property NAME $v]}}
    save DRC_ERRORS_NOT_product.tcllist [dict create actual_error_count [llength $errors] all_error_names $errors]
    set f [open $out/kernel_failing_paths.tsv {WRONLY CREAT EXCL}]
    puts $f "slack\tlogic_levels\tdatapath\tlogic\troute\tskew\tsrc_slr\tdst_slr\tmax_fanout\tsrc\tdst"
    foreach p [get_timing_paths -group [get_clocks clk_kernel_00_unbuffered_net] -max_paths 60000 -nworst 1 -slack_lesser_than 0 -setup] {
      set s [get_property STARTPOINT_PIN $p]; set e [get_property ENDPOINT_PIN $p]
      set mf 0; foreach n [get_nets -of $p] {set fo [get_property FLAT_PIN_COUNT $n]; if {$fo > $mf} {set mf $fo}}
      puts $f "[get_property SLACK $p]\t[get_property LOGIC_LEVELS $p]\t[get_property DATAPATH_DELAY $p]\t[get_property DATAPATH_LOGIC_DELAY $p]\t[get_property DATAPATH_NET_DELAY $p]\t[get_property SKEW $p]\t[get_slrs -of [get_cells -of $s]]\t[get_slrs -of [get_cells -of $e]]\t$mf\t$s\t$e"
    }
    close $f
    set routecode [catch {command ROUTE [list route_design -directive AlternateCLBRouting -ultrathreads]} rresult]
    report ROUTE_STATUS [list report_route_status -file $out/routed_route_status.rpt]
    report ROUTED_TIMING [list report_timing_summary -delay_type min_max -max_paths 20 -file $out/routed_allclock_timing_summary.rpt]
    report ROUTED_CONGESTION [list report_design_analysis -congestion -file $out/routed_congestion.rpt]
    command SAVE_ROUTED [list write_checkpoint $out/salut_ROUTED_${directive}_NOT_product.dcp]
    return [dict create checkpoint $newcp place_code 0 drc_errors [llength $errors] route_code $routecode route_result $rresult]
  }
}
lassign $argv ::place637::subdir ::place637::directive
set ::place637::code [catch {::place637::run $::place637::subdir $::place637::directive} ::place637::result]
if {$::place637::out ne "" && $::place637::placecode ne "NOT_ATTEMPTED" && $::place637::saved eq ""} {
  catch {::place637::command SAVE_FAILED [list write_checkpoint $::place637::out/salut_relay_FAILED_PLACE_NOT_product.dcp]}
}
if {$::place637::out ne ""} {
  ::place637::save STAGE_RESULT_NOT_product.tcllist [dict create actual_stage_run_code $::place637::code actual_result $::place637::result place_code $::place637::placecode saved $::place637::saved]
}
puts [list PLACE637_STAGE $::place637::code $::place637::result]
exit $::place637::code
