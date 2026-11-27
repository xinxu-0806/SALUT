# Reject kernel-only/OOC checkpoints and incomplete kernels in the P&R screen.
proc salut_platform_task_counts {root} {
  set counts {}
  foreach {task expected} {
    ParallelSegmentPE 16 ParallelColumnFront 8 SalutLMLane 8 SalutSmBank 8
    SalutHBMWordPort 16 SalutHBMReadWordPort 8
    SalutReadSwitch2 12 SalutWriteSwitch2 12 SalutResponseSwitch2 12
    SalutPublishJoin 1 SalutMemoryDrainJoin 1
    ParallelScheduler 1 ParallelDegreeStore 1 ParallelPegAckDemux 1
    ParallelPivotReadGate 1
  } {
    set observed 0
    for {set i 0} {$i < $expected} {incr i} {
      set cells [get_cells -quiet ${root}/${task}_$i]
      if {[llength $cells] != 1} {error "SALUT missing or ambiguous actual task ${task}_$i"}
      incr observed [llength $cells]
    }
    dict set counts $task $observed
  }
  return $counts
}
proc salut_assert_full_platform {} {
  set root level0_i/ulp/SparseLUKernel_1/inst
  if {[llength [get_cells -quiet $root]] != 1 ||
      [llength [get_pblocks -quiet pblock_dynamic_region]] != 1} {
    error "SALUT requires the full integrated platform, not an OOC kernel checkpoint"
  }
  # 16 PEs = 8 PEG x 2 (2026-10-22 user-approved 16-PE fallback; was 32 = 8 x 4)
  for {set i 0} {$i < 16} {incr i} {
    if {[llength [get_cells -quiet $root/ParallelSegmentPE_$i]] != 1} {
      error "SALUT full platform is missing PE $i"
    }
  }
  if {[llength [get_cells -quiet $root/ParallelSegmentPE_16]] != 0} {
    error "SALUT full platform has more than the 16 configured PEs"
  }
  # Check every real bank/network actor, not only PE and wrapper counts.
  foreach {task count} {
    ParallelColumnFront 8 SalutLMLane 8 SalutSmBank 8
    SalutHBMWordPort 16 SalutHBMReadWordPort 8
    SalutReadSwitch2 12 SalutWriteSwitch2 12 SalutResponseSwitch2 12
    SalutPublishJoin 1 SalutMemoryDrainJoin 1
    ParallelScheduler 1 ParallelDegreeStore 1 ParallelPegAckDemux 1
    ParallelPivotReadGate 1
  } {
    for {set i 0} {$i < $count} {incr i} {
      if {[llength [get_cells -quiet ${root}/${task}_$i]] != 1} {
        error "SALUT full platform is missing actual task ${task}_$i"
      }
    }
  }
  set ports {}
  foreach family {A L U} {
    for {set i 0} {$i < 8} {incr i} {lappend ports ${family}_$i}
  }
  foreach port $ports {
    if {[llength [get_cells -quiet ${root}/${port}__m_axi]] != 1} {
      error "SALUT full platform is missing HBM AXI wrapper $port"
    }
  }
  # Metadata uses direct HLS AXI interfaces, not sibling TAPA __m_axi cells.
  foreach meta {SCHED FRONT SEGMENT} {
    if {[llength [get_pins -quiet ${root}/m_axi_META_${meta}_mmap_ARADDR*]] == 0} {
      error "SALUT full platform is missing metadata HBM AXI interface $meta"
    }
  }
  puts "SALUT full-platform check: 16 PEs, 8 fronts, 8 LM, 8 SM, real HBM/network/publish/drain and 27 HBM AXI interfaces present"
  return $root
}
