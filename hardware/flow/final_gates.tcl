# v721: READ-ONLY final gates on the v720a routed DCP (fresh reload): timing, DRC, CDC, exceptions, check_timing, bus skew,
# clock interaction, route status, then pr_verify -full_check against the immutable F5. Nothing is written except reports.
set out [file dirname [file normalize [info script]]]
proc stamp {name {msg ""}} {set f [open $::out/$name w]; puts $f "[clock format [clock seconds] -format %Y-%m-%dT%H:%M:%S] $msg"; close $f}
set D build/fpga/banked-lu-dispatch-bram.kpvu1VPp/physical_audits/v720a-LowFreq150PlaceRoute.4D3XriNY/SSI_SpreadSLLs/salut_ROUTED_SSI_SpreadSLLs_NOT_product.dcp
set F5 build/fpga/banked-lu-dispatch-bram.kpvu1VPp/physical_audits/baseline-shell-readonly.full-static-tklyvjwp/original_locked_shell_copy.dcp
if {[lindex [exec sha256sum $F5] 0] ne "f5cee615c973ebd572f85ce6d412edbf273115b8f7b061874d286caa3408bcf3"} {error F5_SHA}
stamp INPUT "[lindex [exec sha256sum $D] 0]"
set t0 [clock seconds]
open_checkpoint $D
stamp RELOADED "elapsed=[expr {[clock seconds]-$t0}]s"
stamp CONSTRAINTS "kernel_period=[get_property PERIOD [get_clocks clk_kernel_00_unbuffered_net]] hbm=[get_property PERIOD [get_clocks hbm_aclk]] HDPR5=[get_property SEVERITY [get_drc_checks HDPR-5]] clocks=[llength [get_clocks]]"
report_route_status -file $out/route_status.rpt
report_timing_summary -delay_type min_max -max_paths 20 -report_unconstrained -file $out/timing_summary.rpt
stamp TIMING "wns=[get_property SLACK [get_timing_paths -setup]] whs=[get_property SLACK [get_timing_paths -hold]]"
catch {report_pulse_width -file $out/pulse_width.rpt}
report_drc -file $out/drc_default.rpt
set sev [dict create]; foreach v [get_drc_violations] { dict incr sev [get_property SEVERITY $v] }
stamp DRC "severity_counts=$sev"
report_cdc -details -file $out/cdc.rpt
report_clock_interaction -delay_type min_max -file $out/clock_interaction.rpt
report_exceptions -ignored -file $out/exceptions.rpt
check_timing -verbose -file $out/check_timing.rpt
report_bus_skew -file $out/bus_skew.rpt
report_clocks -file $out/clocks.rpt
report_disable_timing -file $out/disabled_timing.rpt
stamp REPORTS_DONE
set code [catch {pr_verify -full_check -in_memory -additional [list $F5] -file $out/pr_verify_full_F5.rpt} result options]
stamp PR_VERIFY "code=$code result=$result"
stamp DONE
