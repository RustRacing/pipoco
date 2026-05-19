---- MODULE ts_proto ----
EXTENDS Integers, TLC

DispatchStates == {"Idle", "RxFrame", "Decode", "Execute", "EncodeReply", "ErrorReply"}
RecognizedCommands == {"ReadPage", "WritePage", "Burn", "GetOutpc", "GetSignature"}
AllCommands == RecognizedCommands \cup {"Unknown"}

VARIABLES state, pending_command, response_ready, error_reply
Vars == << state, pending_command, response_ready, error_reply >>

Init ==
    /\ state = "Idle"
    /\ pending_command = "Unknown"
    /\ response_ready = FALSE
    /\ error_reply = FALSE

IdleToRxRecognized ==
    /\ state = "Idle"
    /\ \E cmd \in RecognizedCommands :
        /\ state' = "RxFrame"
        /\ pending_command' = cmd
    /\ response_ready' = FALSE
    /\ error_reply' = FALSE

IdleToRxUnknown ==
    /\ state = "Idle"
    /\ state' = "RxFrame"
    /\ pending_command' = "Unknown"
    /\ response_ready' = FALSE
    /\ error_reply' = FALSE

RxToDecode ==
    /\ state = "RxFrame"
    /\ state' = "Decode"
    /\ UNCHANGED << pending_command, response_ready, error_reply >>

DecodeMalformed ==
    /\ state = "Decode"
    /\ state' = "ErrorReply"
    /\ pending_command' = "Unknown"
    /\ response_ready' = FALSE
    /\ error_reply' = TRUE

DecodeToExecute ==
    /\ state = "Decode"
    /\ pending_command \in AllCommands
    /\ state' = "Execute"
    /\ response_ready' = FALSE
    /\ error_reply' = FALSE
    /\ UNCHANGED pending_command

ExecuteRecognized ==
    /\ state = "Execute"
    /\ pending_command \in RecognizedCommands
    /\ state' = "EncodeReply"
    /\ response_ready' = TRUE
    /\ error_reply' = FALSE
    /\ UNCHANGED pending_command

ExecuteUnknown ==
    /\ state = "Execute"
    /\ pending_command = "Unknown"
    /\ state' = "ErrorReply"
    /\ response_ready' = FALSE
    /\ error_reply' = TRUE
    /\ UNCHANGED pending_command

EncodeToIdle ==
    /\ state = "EncodeReply"
    /\ state' = "Idle"
    /\ pending_command' = "Unknown"
    /\ response_ready' = FALSE
    /\ error_reply' = FALSE

ErrorToIdle ==
    /\ state = "ErrorReply"
    /\ state' = "Idle"
    /\ pending_command' = "Unknown"
    /\ response_ready' = FALSE
    /\ error_reply' = FALSE

Stutter ==
    /\ UNCHANGED Vars

Next ==
    \/ IdleToRxRecognized
    \/ IdleToRxUnknown
    \/ RxToDecode
    \/ DecodeMalformed
    \/ DecodeToExecute
    \/ ExecuteRecognized
    \/ ExecuteUnknown
    \/ EncodeToIdle
    \/ ErrorToIdle
    \/ Stutter

ResponseStateConsistent ==
    response_ready => state \in {"EncodeReply", "Idle"}

ErrorStateConsistent ==
    error_reply => state \in {"ErrorReply", "Idle"}

Inv ==
    /\ state \in DispatchStates
    /\ pending_command \in AllCommands
    /\ response_ready \in BOOLEAN
    /\ error_reply \in BOOLEAN
    /\ ResponseStateConsistent
    /\ ErrorStateConsistent

RecognizedPending ==
    state = "Execute" /\ pending_command \in RecognizedCommands

ConditionalLiveness ==
    [](RecognizedPending => <> (state = "EncodeReply" /\ response_ready))

Spec ==
    /\ Init
    /\ [][Next]_Vars
    /\ WF_Vars(DecodeToExecute)
    /\ WF_Vars(ExecuteRecognized)

====
