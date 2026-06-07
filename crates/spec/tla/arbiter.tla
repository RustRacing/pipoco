---- MODULE arbiter ----
EXTENDS Integers, TLC

PriorityCodes == 0..7

VARIABLES safety_latched, hard_rev_cut, launch_cut, flat_shift_cut, dfco_cut, soft_rev_cut, knock_active, cut_reason_code
Vars == << safety_latched, hard_rev_cut, launch_cut, flat_shift_cut, dfco_cut, soft_rev_cut, knock_active, cut_reason_code >>

Init ==
    /\ safety_latched = FALSE
    /\ hard_rev_cut = FALSE
    /\ launch_cut = FALSE
    /\ flat_shift_cut = FALSE
    /\ dfco_cut = FALSE
    /\ soft_rev_cut = FALSE
    /\ knock_active = FALSE
    /\ cut_reason_code = 0

ResolvedCode(s, h, l, f, d, sr, k) ==
    IF s THEN 1
    ELSE IF h THEN 2
    ELSE IF l THEN 3
    ELSE IF f THEN 4
    ELSE IF d THEN 5
    ELSE IF sr THEN 6
    ELSE IF k THEN 7
    ELSE 0

AcquireInput ==
    \E s \in BOOLEAN,
      h \in BOOLEAN,
      l \in BOOLEAN,
      f \in BOOLEAN,
      d \in BOOLEAN,
      sr \in BOOLEAN,
      k \in BOOLEAN :
        /\ safety_latched' = s
        /\ hard_rev_cut' = h
        /\ launch_cut' = l
        /\ flat_shift_cut' = f
        /\ dfco_cut' = d
        /\ soft_rev_cut' = sr
        /\ knock_active' = k
        /\ cut_reason_code' = ResolvedCode(s, h, l, f, d, sr, k)

Next == AcquireInput

HigherPriorityAsserted(code) ==
    CASE code = 1 -> FALSE
      [] code = 2 -> safety_latched
      [] code = 3 -> safety_latched \/ hard_rev_cut
      [] code = 4 -> safety_latched \/ hard_rev_cut \/ launch_cut
      [] code = 5 -> safety_latched \/ hard_rev_cut \/ launch_cut \/ flat_shift_cut
      [] code = 6 -> safety_latched \/ hard_rev_cut \/ launch_cut \/ flat_shift_cut \/ dfco_cut
      [] code = 7 -> safety_latched \/ hard_rev_cut \/ launch_cut \/ flat_shift_cut \/ dfco_cut \/ soft_rev_cut
      [] OTHER -> FALSE

PriorityOrderedResolution ==
    /\ cut_reason_code = ResolvedCode(safety_latched, hard_rev_cut, launch_cut, flat_shift_cut, dfco_cut, soft_rev_cut, knock_active)
    /\ cut_reason_code \in PriorityCodes

NoLowerPriorityCutWins ==
    \A code \in 1..7 :
        cut_reason_code = code => ~HigherPriorityAsserted(code)

Inv ==
    /\ PriorityOrderedResolution
    /\ NoLowerPriorityCutWins

Spec ==
    /\ Init
    /\ [][Next]_Vars

====
