---- MODULE trigger ----
EXTENDS Integers, TLC

SyncState == {"Unsynced", "PreSync", "Synced", "SyncLoss"}
MaxGood == 2
MaxBad == 2

VARIABLES sync_state, noise_present, consecutive_good, consecutive_bad
Vars == << sync_state, noise_present, consecutive_good, consecutive_bad >>

Init ==
    /\ sync_state = "Unsynced"
    /\ noise_present = TRUE
    /\ consecutive_good = 0
    /\ consecutive_bad = 0

GoodToothArrival ==
    /\ sync_state' = sync_state
    /\ consecutive_good' = IF consecutive_good < MaxGood THEN consecutive_good + 1 ELSE MaxGood
    /\ consecutive_bad' = 0
    /\ noise_present' = noise_present

BadToothArrival ==
    /\ noise_present = TRUE
    /\ sync_state' = sync_state
    /\ consecutive_good' = 0
    /\ consecutive_bad' = IF consecutive_bad < MaxBad THEN consecutive_bad + 1 ELSE MaxBad
    /\ noise_present' = noise_present

SyncAcquire ==
    /\ sync_state \in {"Unsynced", "PreSync", "SyncLoss"}
    /\ consecutive_good = MaxGood
    /\ sync_state' = "Synced"
    /\ consecutive_good' = consecutive_good
    /\ consecutive_bad' = consecutive_bad
    /\ noise_present' = noise_present

SyncLoss ==
    /\ sync_state = "Synced"
    /\ consecutive_bad = MaxBad
    /\ sync_state' = "SyncLoss"
    /\ consecutive_good' = consecutive_good
    /\ consecutive_bad' = consecutive_bad
    /\ noise_present' = noise_present

ResyncTrack ==
    /\ sync_state = "SyncLoss"
    /\ noise_present = FALSE
    /\ sync_state' = "PreSync"
    /\ consecutive_good' = consecutive_good
    /\ consecutive_bad' = consecutive_bad
    /\ noise_present' = noise_present

NoiseClears ==
    /\ noise_present = TRUE
    /\ noise_present' = FALSE
    /\ sync_state' = sync_state
    /\ consecutive_good' = consecutive_good
    /\ consecutive_bad' = 0

Stutter ==
    /\ UNCHANGED Vars

Next ==
    \/ GoodToothArrival
    \/ BadToothArrival
    \/ SyncAcquire
    \/ SyncLoss
    \/ ResyncTrack
    \/ NoiseClears
    \/ Stutter

CountersBounded ==
    /\ consecutive_good \in 0..MaxGood
    /\ consecutive_bad \in 0..MaxBad

NoBadCountWhenNoiseRemoved ==
    noise_present = FALSE => consecutive_bad = 0

Inv ==
    /\ sync_state \in SyncState
    /\ noise_present \in BOOLEAN
    /\ CountersBounded
    /\ NoBadCountWhenNoiseRemoved

NoiseRemovedAssumption == <>[] (noise_present = FALSE)

ConditionalLiveness ==
    NoiseRemovedAssumption => <>(sync_state = "Synced")

Spec ==
    /\ Init
    /\ [][Next]_Vars
    /\ WF_Vars(GoodToothArrival)
    /\ WF_Vars(SyncAcquire)

====
