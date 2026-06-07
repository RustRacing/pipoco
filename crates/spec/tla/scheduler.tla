---- MODULE scheduler ----
EXTENDS Integers, Sequences, FiniteSets, TLC

Mode == {"Off", "Cranking", "Running", "Shutdown"}
Sync == {"Unsynced", "Synced"}
Angles == 0..1
RpmBuckets == 0..1
MaxPending == 4
MaxOutputs == 4
Cylinders == {0}
EventKind == {"InjectionOpen", "InjectionClose", "CoilChargeStart", "CoilFire"}

Event == [kind : EventKind, cylinder : Cylinders, angle : Angles]
Output == [kind : EventKind, cylinder : Cylinders]

VARIABLES mode, sync, rpm, theta, fuel_cut, spark_cut, pending, outputs, pw
Vars == << mode, sync, rpm, theta, fuel_cut, spark_cut, pending, outputs, pw >>

SeenIn(seq, kind, cyl) ==
    \E i \in 1..Len(seq) : seq[i].kind = kind /\ seq[i].cylinder = cyl

SeenOutput(kind, cyl) ==
    [kind |-> kind, cylinder |-> cyl] \in outputs

Seen(kind, cyl) ==
    SeenIn(pending, kind, cyl) \/ SeenOutput(kind, cyl)

Init ==
    /\ mode = "Off"
    /\ sync = "Unsynced"
    /\ rpm = 0
    /\ theta = 0
    /\ fuel_cut = FALSE
    /\ spark_cut = FALSE
    /\ pending = << >>
    /\ outputs = {}
    /\ pw = 0

ValidEvent(e) ==
    /\ e.kind \in EventKind
    /\ e.cylinder \in Cylinders
    /\ e.angle \in Angles

CanInject ==
    /\ mode \in {"Cranking", "Running"}
    /\ sync = "Synced"
    /\ fuel_cut = FALSE
    /\ Len(pending) + 2 <= MaxPending
    /\ \E cyl \in Cylinders :
        /\ ~(Seen("InjectionOpen", cyl) /\ Seen("InjectionClose", cyl))

CanSpark ==
    /\ mode \in {"Cranking", "Running"}
    /\ sync = "Synced"
    /\ spark_cut = FALSE
    /\ Len(pending) + 2 <= MaxPending
    /\ \E cyl \in Cylinders :
        /\ ~(Seen("CoilChargeStart", cyl) /\ Seen("CoilFire", cyl))

AcquireInput ==
    /\ mode' \in Mode
    /\ sync' \in Sync
    /\ rpm' \in RpmBuckets
    /\ fuel_cut' \in BOOLEAN
    /\ spark_cut' \in BOOLEAN
    /\ pending' =
        IF sync' = "Unsynced"
            THEN << >>
            ELSE pending
    /\ fuel_cut' => \A i \in 1..Len(pending') : pending'[i].kind \notin {"InjectionOpen", "InjectionClose"}
    /\ spark_cut' => \A i \in 1..Len(pending') : pending'[i].kind \notin {"CoilChargeStart", "CoilFire"}
    /\ UNCHANGED << theta, outputs, pw >>

EvalFuel ==
    /\ pw' \in 0..20
    /\ UNCHANGED << mode, sync, rpm, theta, fuel_cut, spark_cut, pending, outputs >>

ScheduleInjection ==
    /\ CanInject
    /\ pending' =
        Append(
            Append(pending, [kind |-> "InjectionOpen", cylinder |-> 0, angle |-> theta]),
            [kind |-> "InjectionClose", cylinder |-> 0, angle |-> ((theta + 1) % 2)]
        )
    /\ UNCHANGED << mode, sync, rpm, theta, fuel_cut, spark_cut, outputs, pw >>

ScheduleSpark ==
    /\ CanSpark
    /\ pending' =
        Append(
            Append(pending, [kind |-> "CoilChargeStart", cylinder |-> 0, angle |-> theta]),
            [kind |-> "CoilFire", cylinder |-> 0, angle |-> ((theta + 1) % 2)]
        )
    /\ UNCHANGED << mode, sync, rpm, theta, fuel_cut, spark_cut, outputs, pw >>

FireEvent ==
    /\ Len(pending) > 0
    /\ Cardinality(outputs) < MaxOutputs
    /\ LET e == Head(pending) IN
        outputs' = outputs \cup {[kind |-> e.kind, cylinder |-> e.cylinder]}
    /\ pending' = Tail(pending)
    /\ UNCHANGED << mode, sync, rpm, theta, fuel_cut, spark_cut, pw >>

AdvanceAngle ==
    /\ theta' = (theta + 1) % 2
    /\ UNCHANGED << mode, sync, rpm, fuel_cut, spark_cut, pending, outputs, pw >>

Next ==
    \/ AcquireInput
    \/ EvalFuel
    \/ ScheduleInjection
    \/ ScheduleSpark
    \/ FireEvent
    \/ AdvanceAngle

NoNegativePW == pw >= 0
AnglesInRange == theta \in Angles
PendingBounded == Len(pending) <= MaxPending
OutputsBounded ==
    /\ outputs \subseteq Output
    /\ Cardinality(outputs) <= MaxOutputs

NoInjectionWhenFuelCut ==
    fuel_cut => \A i \in 1..Len(pending) : pending[i].kind \notin {"InjectionOpen", "InjectionClose"}

NoSparkWhenSparkCut ==
    spark_cut => \A i \in 1..Len(pending) : pending[i].kind \notin {"CoilChargeStart", "CoilFire"}

SparkAfterDwell ==
    \A e \in outputs :
        e.kind = "CoilFire" =>
            [kind |-> "CoilChargeStart", cylinder |-> e.cylinder] \in outputs

DeterministicStep ==
    /\ pending \in Seq(Event)
    /\ outputs \subseteq Output
    /\ \A i \in 1..Len(pending) : ValidEvent(pending[i])

Inv ==
    /\ NoNegativePW
    /\ AnglesInRange
    /\ NoInjectionWhenFuelCut
    /\ NoSparkWhenSparkCut
    /\ SparkAfterDwell
    /\ DeterministicStep
    /\ PendingBounded
    /\ OutputsBounded

StableRunning ==
    <>[](
        mode = "Running" /\
        sync = "Synced" /\
        fuel_cut = FALSE /\
        spark_cut = FALSE /\
        rpm # 0
    )

Emitted(kind, cyl) ==
    [kind |-> kind, cylinder |-> cyl] \in outputs

AllKindsEventually(cyl) ==
    /\ <>Emitted("InjectionOpen", cyl)
    /\ <>Emitted("InjectionClose", cyl)
    /\ <>Emitted("CoilChargeStart", cyl)
    /\ <>Emitted("CoilFire", cyl)

PendingClearsWhenUnsynced ==
    []((sync = "Unsynced" /\ pending # << >>) => <>(pending = << >>))

ConditionalLiveness ==
    /\ PendingClearsWhenUnsynced
    /\ StableRunning => \A cyl \in Cylinders : AllKindsEventually(cyl)

Spec ==
    /\ Init
    /\ [][Next]_Vars
    /\ WF_Vars(AdvanceAngle)
    /\ WF_Vars((Len(pending) > 0) /\ FireEvent)
    /\ WF_Vars(CanInject /\ ScheduleInjection)
    /\ WF_Vars(CanSpark /\ ScheduleSpark)

====
