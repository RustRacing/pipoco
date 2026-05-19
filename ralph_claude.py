#!/usr/bin/env python3
import json
import os
import re
import shutil
import signal
import subprocess
import sys
from pathlib import Path

import ralph_simple as rs


CLAUDE_SESSION_ID_PATTERN = re.compile(r'"session_id":"([0-9a-fA-F-]{36})"')
CLAUDE_RESUME_INVALID_PATTERNS = (
    "No conversation found with session ID:",
)


def claude_settings():
    claude_bin = os.environ.get("CLAUDE_BIN") or shutil.which("claude")
    if not claude_bin:
        rs.eprint("Error: claude not found on PATH. Set CLAUDE_BIN=/path/to/claude.")
        sys.exit(1)
    model = os.environ.get("CLAUDE_MODEL")
    effort = os.environ.get("CLAUDE_EFFORT")
    permission_mode = os.environ.get("CLAUDE_PERMISSION_MODE", "bypassPermissions")
    return claude_bin, model, effort, permission_mode


def with_planning_suffix(prompt: str) -> str:
    return prompt + "\n PLAN BEFORE PROCEEDING\n REDUCE ROUND TRIPS, DO PARALLEL TOOL CALLING!!"


def build_claude_args(prompt: str, effort_override: str | None = None):
    claude_bin, model, effort, permission_mode = claude_settings()
    args = [
        claude_bin,
        "-p",
        "--verbose",
        "--output-format",
        "stream-json",
        "--permission-mode",
        permission_mode,
    ]
    if model:
        args.extend(["--model", model])
    effective_effort = effort_override or effort
    if effective_effort:
        args.extend(["--effort", effective_effort])
    args.append(with_planning_suffix(prompt))
    return args, (model or "(claude default)"), (effective_effort or "(claude default)")


def build_claude_resume_args(session_id: str, prompt: str, effort_override: str | None = None):
    claude_bin, model, effort, permission_mode = claude_settings()
    args = [
        claude_bin,
        "-p",
        "--verbose",
        "--output-format",
        "stream-json",
        "--permission-mode",
        permission_mode,
        "-r",
        session_id,
    ]
    if model:
        args.extend(["--model", model])
    effective_effort = effort_override or effort
    if effective_effort:
        args.extend(["--effort", effective_effort])
    args.append(with_planning_suffix(prompt))
    return args


class ClaudeRunner(rs.Runner):
    def mark_story_blocked(self, story_id: str, note: str):
        prd = rs.load_prd(self.prd_file)
        changed = False
        for story in rs.prd_stories(prd):
            if story.get("id") != story_id or story.get("passes", False):
                continue
            story["status"] = "blocked"
            existing = story.get("notes")
            story["notes"] = f"{existing}\n{note}" if existing else note
            changed = True
            break
        if changed:
            self.prd_file.write_text(json.dumps(prd, indent=2) + "\n")
        return changed

    def run(self):
        self.ensure_files()
        _args, model, effort = build_claude_args("ping")

        print("=== Ralph Loop ===")
        print(f"PRD: {self.prd_file.name}")
        print(f"Model: {model}")
        print(f"Effort: {effort}")
        print(f"Session mode: {'continuable' if self.continue_mode else 'ephemeral'}")
        print(f"Blocked retry limit: {self.blocked_retry_limit}")
        print(f"Continue mode: {'on' if self.continue_mode else 'off'}")
        context_file = self.resolved_context_file(rs.load_prd(self.prd_file))
        print(f"Context file: {rs.display_path(context_file) if context_file else '(none)'}")
        print()

        signal.signal(signal.SIGINT, self.handle_signal)
        signal.signal(signal.SIGTERM, self.handle_signal)

        for iteration in range(1, self.max_iterations + 1):
            prd = rs.load_prd(self.prd_file)
            current_prd_sig = rs.prd_signature(self.prd_file, prd)
            pre_attempt_pass_states = rs.prd_pass_states(prd)
            if rs.all_stories_pass(prd):
                print("All stories complete!")
                return 0

            blocked_attempt = False
            story = rs.first_actionable_story(prd)
            if story is None:
                blocked_story = rs.first_incomplete_story(prd)
                if blocked_story is not None:
                    if self.should_stop_on_blocked(blocked_story):
                        return 2
                    print(
                        "No actionable incomplete stories remain in this iteration. "
                        f"First incomplete story is {blocked_story.get('id')} "
                        f"with status={blocked_story.get('status', 'pending')}. "
                        "Switching to blocker prompt."
                    )
                    print()
                    story = blocked_story
                    blocked_attempt = True
                else:
                    print("All stories complete!")
                    return 0

            story_id = story.get("id", "(unknown)")
            story_title = story.get("title", "")
            print(f"=== Iteration {iteration}: {story_id} — {story_title}")
            blocked_story_before = rs.story_fingerprint(story) if blocked_attempt else None

            session_id = self.load_session_id() if self.continue_mode else None
            if self.continue_mode and session_id:
                saved_meta = self.load_session_meta()
                if saved_meta != current_prd_sig:
                    print(
                        "Stored continuation session belongs to a different PRD. "
                        "Clearing Claude session and starting this batch fresh."
                    )
                    self.clear_session_id()
                    session_id = None

            attempted_resume = bool(self.continue_mode and session_id)
            while True:
                blocked_effort = os.environ.get("CLAUDE_BLOCKED_EFFORT", "medium")
                prompt = (
                    self.blocker_prompt_for(story)
                    if blocked_attempt
                    else (self.continue_prompt_for(story) if attempted_resume else self.prompt_for())
                )
                if attempted_resume:
                    iter_args = build_claude_resume_args(
                        session_id,
                        prompt,
                        blocked_effort if blocked_attempt else None,
                    )
                else:
                    iter_args = build_claude_args(
                        prompt,
                        blocked_effort if blocked_attempt else None,
                    )[0]

                self.current_proc = subprocess.Popen(
                    iter_args,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid,
                    text=True,
                    bufsize=1,
                )
                assert self.current_proc.stdout is not None

                seen_session_id = session_id if attempted_resume else None
                output_lines = []
                for line in self.current_proc.stdout:
                    output_lines.append(line)
                    sys.stdout.write(line)
                    sys.stdout.flush()
                    if self.continue_mode and not seen_session_id:
                        match = CLAUDE_SESSION_ID_PATTERN.search(line)
                        if match:
                            seen_session_id = match.group(1)
                rc = self.current_proc.wait()
                self.current_proc = None

                combined_output = "".join(output_lines)
                if attempted_resume and any(p in combined_output for p in CLAUDE_RESUME_INVALID_PATTERNS):
                    print("Stored Claude session is invalid. Clearing it and retrying this iteration with a fresh prompt.")
                    self.clear_session_id()
                    session_id = None
                    attempted_resume = False
                    continue
                break

            if rc != 0:
                print(f"Claude exited with status {rc}.")
                self.run_prune()
                print()
                return rc

            if self.continue_mode and seen_session_id:
                self.save_session_id(seen_session_id)
                self.save_session_meta(prd)

            prd_after = rs.load_prd(self.prd_file)
            post_attempt_pass_states = rs.prd_pass_states(prd_after)
            story_after = next(
                (s for s in rs.prd_stories(prd_after) if s.get("id") == story_id),
                None,
            )
            if story_after and (story_after.get("passes") or story_after.get("status") == "blocked"):
                self.clear_retry_state()
            elif post_attempt_pass_states == pre_attempt_pass_states:
                retry_state = self.load_retry_state()
                if (
                    retry_state
                    and retry_state.get("story_id") == story_id
                    and retry_state.get("pass_states") == pre_attempt_pass_states
                ):
                    stagnant_count = int(retry_state.get("count", 0)) + 1
                else:
                    stagnant_count = 1
                self.save_retry_state(story_id, pre_attempt_pass_states, stagnant_count)
                print(
                    f"Story {story_id} left PRD pass/fail state unchanged "
                    f"({stagnant_count}/3)."
                )
                if stagnant_count >= 3 and story_after and not story_after.get("passes", False):
                    note = (
                        f"Auto-blocked by ralph_claude.py on {subprocess.check_output(['date', '+%F'], text=True).strip()}: "
                        "same story attempted three times with unchanged PRD pass/fail state."
                    )
                    if self.mark_story_blocked(story_id, note):
                        self.clear_retry_state()
                        print(
                            f"Story {story_id} marked blocked after 3 unchanged pass/fail attempts."
                        )
            else:
                self.clear_retry_state()

            if blocked_attempt and blocked_story_before is not None:
                blocked_story_after = rs.first_incomplete_story(prd_after)
                if rs.story_fingerprint(blocked_story_after) != blocked_story_before:
                    if story_id in self.blocked_retries:
                        self.blocked_retries[story_id] = 0
                    print(
                        f"Blocked story {story_id} changed after this attempt. "
                        "Resetting blocked retry counter."
                    )

            print()
            print(f"Iteration {iteration} complete.")
            self.run_prune()
            print()

        print(f"Reached max iterations ({self.max_iterations}). Stopping.")
        return 1


def main(argv: list[str]) -> int:
    prd_file, continue_mode, context_file = rs.parse_runner_cli(argv, "prd.json")
    runner = ClaudeRunner(
        prd_file=prd_file,
        progress_file=Path("progress.txt"),
        max_iterations=int(os.environ.get("MAX_ITERATIONS", "300")),
        blocked_retry_limit=int(os.environ.get("BLOCKED_RETRY_LIMIT", "3")),
        continue_mode=continue_mode,
        session_file=Path(".claude_session_id"),
        session_meta_file=Path(".claude_session_meta.json"),
        retry_state_file=Path(".ralph_claude_retry_state.json"),
        context_file=context_file,
    )
    return runner.run()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
