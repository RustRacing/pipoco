---- MODULE persist ----
EXTENDS Integers, Sequences, TLC

Byte == 0..255
CommittedStates == {"none", "commit", "rollback"}
SymbolicPayloads == {
    <<>>,
    <<0>>,
    <<1, 2, 3>>,
    <<10, 20, 30, 40>>,
    <<255, 0, 1, 2, 3>>
}

VARIABLES in_txn, engine_running, staged_bytes, staged_crc, committed_bytes, committed_crc, last_resolution
Vars == << in_txn, engine_running, staged_bytes, staged_crc, committed_bytes, committed_crc, last_resolution >>

Crc(bytes) == Len(bytes) % 256

Init ==
    /\ in_txn = FALSE
    /\ engine_running = FALSE
    /\ staged_bytes = << >>
    /\ staged_crc = 0
    /\ committed_bytes = << >>
    /\ committed_crc = 0
    /\ last_resolution = "none"

BeginTxn ==
    /\ ~in_txn
    /\ in_txn' = TRUE
    /\ staged_bytes' = committed_bytes
    /\ staged_crc' = committed_crc
    /\ UNCHANGED << engine_running, committed_bytes, committed_crc, last_resolution >>

WriteStage ==
    /\ in_txn
    /\ \E bytes \in SymbolicPayloads :
        /\ staged_bytes' = bytes
        /\ staged_crc' = Crc(bytes)
    /\ UNCHANGED << in_txn, engine_running, committed_bytes, committed_crc, last_resolution >>

SetEngineRunning ==
    /\ \E running \in BOOLEAN :
        /\ engine_running' = running
    /\ UNCHANGED << in_txn, staged_bytes, staged_crc, committed_bytes, committed_crc, last_resolution >>

CommitTxn ==
    /\ in_txn
    /\ ~engine_running
    /\ staged_crc = Crc(staged_bytes)
    /\ in_txn' = FALSE
    /\ committed_bytes' = staged_bytes
    /\ committed_crc' = staged_crc
    /\ last_resolution' = "commit"
    /\ UNCHANGED << engine_running, staged_bytes, staged_crc >>

RollbackTxn ==
    /\ in_txn
    /\ in_txn' = FALSE
    /\ staged_bytes' = committed_bytes
    /\ staged_crc' = committed_crc
    /\ last_resolution' = "rollback"
    /\ UNCHANGED << engine_running, committed_bytes, committed_crc >>

Stutter ==
    /\ UNCHANGED Vars

Next ==
    \/ BeginTxn
    \/ WriteStage
    \/ SetEngineRunning
    \/ CommitTxn
    \/ RollbackTxn
    \/ Stutter

CommittedCrcConsistent ==
    committed_crc = Crc(committed_bytes)

Inv ==
    /\ in_txn \in BOOLEAN
    /\ engine_running \in BOOLEAN
    /\ staged_bytes \in SymbolicPayloads
    /\ staged_crc \in 0..255
    /\ committed_bytes \in SymbolicPayloads
    /\ committed_crc \in 0..255
    /\ last_resolution \in CommittedStates
    /\ CommittedCrcConsistent

ConditionalLiveness ==
    [](in_txn => <>(~in_txn /\ last_resolution \in {"commit", "rollback"}))

Spec ==
    /\ Init
    /\ [][Next]_Vars
    /\ WF_Vars(CommitTxn)
    /\ WF_Vars(RollbackTxn)

====
