# PLACE_DESIGN PRE: save the complete platform after opt_design, before
# task floorplanning/placement. Deliberately stop this capture-only v++ run.
source [file join [file dirname [info script]] assert_full_platform.tcl]
salut_assert_full_platform
if {![info exists ::env(SALUT_POSTOPT_DCP)]} {
  error "SALUT_POSTOPT_DCP must name a new capture output"
}
set output $::env(SALUT_POSTOPT_DCP)
if {[file exists $output] || [file exists $output.ready]} {
  error "SALUT refuses to overwrite an existing post-opt capture"
}
write_checkpoint $output
if {![file exists $output] || [file size $output] == 0} {
  error "SALUT post-opt checkpoint was not saved"
}
set marker [open $output.ready {WRONLY CREAT EXCL}]
puts $marker SALUT_CAPTURE_ONLY_COMPLETE
close $marker
error SALUT_CAPTURE_ONLY_COMPLETE
