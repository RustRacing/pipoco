---- MODULE safety ----
EXTENDS Integers, TLC

PriorityCodes == 0..7

VARIABLES safety_latched, fault_overtemp, fault_oil_pressure, clear_condition, cut_reason_code, fuel_cut, spark_cut
Vars == << safety_latched, fault_overtemp, fault_oil_pressure, clear_condition, cut_reason_code, fuel_cut, spark_cut >>

Init ==
    /\ safety_latched = FALSE
    /\ fault_overtemp = FALSE
    /\ fault_oil_pressure = FALSE
    /\ clear_condition = FALSE
    /\ cut_reason_code = 0
    /\ fuel_cut = FALSE
    /\ spark_cut = FALSE

AnyLatchFault == fault_overtemp \/ fault_oil_pressure
ClearConditionHolds == clear_condition /\ ~AnyLatchFault

AcquireInput ==
    /\ fault_overtemp' \in BOOLEAN
    /\ fault_oil_pressure' \in BOOLEAN
    /\ clear_condition' \in BOOLEAN
    /\ UNCHANGED << safety_latched, cut_reason_code, fuel_cut, spark_cut >>

NextLatched ==
    IF AnyLatchFault THEN TRUE
    ELSE IF safety_latched /\ ClearConditionHolds THEN FALSE
    ELSE safety_latched

StepSafety ==
    /\ safety_latched' = NextLatched
    /\ cut_reason_code' = IF NextLatched THEN 1 ELSE 0
    /\ fuel_cut' = IF NextLatched THEN TRUE ELSE FALSE
    /\ spark_cut' = IF NextLatched THEN TRUE ELSE FALSE
    /\ UNCHANGED << fault_overtemp, fault_oil_pressure, clear_condition >>

Stutter ==
    /\ UNCHANGED Vars

Next ==
    \/ AcquireInput
    \/ StepSafety
    \/ Stutter

LatchedImpliesCutAsserted ==
    safety_latched => /\ cut_reason_code = 1
                      /\ fuel_cut = TRUE
                      /\ spark_cut = TRUE

CutCodeDomain == cut_reason_code \in PriorityCodes

Inv ==
    /\ safety_latched \in BOOLEAN
    /\ fault_overtemp \in BOOLEAN
    /\ fault_oil_pressure \in BOOLEAN
    /\ clear_condition \in BOOLEAN
    /\ fuel_cut \in BOOLEAN
    /\ spark_cut \in BOOLEAN
    /\ CutCodeDomain
    /\ LatchedImpliesCutAsserted

ConditionalLiveness ==
    []((safety_latched /\ <>[]ClearConditionHolds) => <> (~safety_latched))

Spec ==
    /\ Init
    /\ [][Next]_Vars
    /\ WF_Vars(StepSafety)

====
