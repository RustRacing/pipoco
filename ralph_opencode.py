#!/usr/bin/env python3
"""
Ralph loop using OpenCode instead of Codex.

Usage:
    ./ralph_opencode.py [--continue] [prd_file]

Same interface as ralph_simple.py, but uses `opencode run --format json`
instead of `codex exec`. All PRD state files (progress.txt, .session_meta.json,
.ralph_retry_state.json) are shared with the Codex version for safe switching.

Environment variables:
    OPENCODE_BIN      path to opencode binary (default: search PATH)
    OPENCODE_MODEL    model to use (default: provider/model from opencode config)
    OPENCODE_DIR      working directory for opencode (default: current directory)
"""
import json
import os
import re
import shutil
import signal
import subprocess
import sys
from pathlib import Path

PROMPT_TEMPLATE = r"""You are Ralph, an autonomous coding agent. You execute user stories from the PRD file one at a time.

## Instructions

1. Read the PRD file to understand the project and find the next incomplete story
   (first story where `"passes": false`, in priority order).
2. If all stories have `"passes": true`, say "All stories complete." and exit.
3. For the chosen story:
   a. Read its description and acceptance criteria carefully.
   b. Implement the changes needed to satisfy ALL acceptance criteria.
   c. After implementing, verify each acceptance criterion is met
      (run compiler checks, lints, and tests as specified).
   d. If all criteria pass, update the PRD file — set that story's `"passes": true`
      and add relevant notes.
   e. Append a short summary of what you did to `progress.txt`.
4. Only work on ONE story per invocation. After completing (or failing) one story, stop.
5. If you cannot complete a story, set its `"notes"` field with what went wrong and stop.
   Do NOT mark it as passing.
6. Commit your changes with a message referencing the story ID
   (e.g., "US-001: Add status column to tasks table").

## Important
- You have no memory of previous iterations. Read the PRD and progress.txt to
  understand current state.
- Since a task can be long, do not give up on compiler errors. Use your rust skill.
- Large refactors are allowed, but plan them well.
- Do not skip acceptance criteria. Every criterion must be verified.
- Do not work on more than one story.
- After finishing the story (pass or fail), stop immediately.
- Any new markdown documentation goes to aidocs/ on the project root. Do not commit those.
- AVOID A LOT OF REQUESTS. Plan first and do multiple tool calls per round. Plan around the
    least amount of requests/iterations that are possible. You can think a lot before returning
    but we are charged by request or round-trip, so be sure to plan the next step very well.
- End each task with `make prune`
"""

CONTINUE_PROMPT_TEMPLATE = (
    "Continue this batch. Read the PRD and progress log again, then work only on this "
    "already-selected pending actionable story: __STORY_ID__ — __STORY_TITLE__."
    "__STORY_DESCRIPTION_SEGMENT__"
    " Do not pick a different story and do not pick a blocked story under this prompt; "
    "blocked stories are handled only by the blocker prompt path."
    " If this pending task is not making progress, mark it as blocked for replanning."
    " Otherwise complete exactly this one story and stop."
    " Use subagents whenever possible and use the rust skill when dealing with Rust code."
)

BLOCKER_PROMPT_TEMPLATE = r"""You are Ralph, an autonomous coding agent. You are handling a blocked story from the PRD.

Blocked story: __STORY_ID__ — __STORY_TITLE__

## Instructions

1. Read the PRD file and `progress.txt`.
2. Focus on the first incomplete story, which is currently blocked.
3. Your job is to unblock forward progress, not just to gather one more data point.
4. You may, if justified by evidence:
   a. implement lower-level protocol/runtime fixes
   b. reduce the blocker into deterministic tests or replay fixtures
   c. split the blocked story into smaller prerequisite stories
   d. insert new prerequisite stories before the blocked story
   e. update dependencies so the next run has smaller actionable work
5. Prefer creating many future actionable steps over one more broad rerun.
6. If you fully satisfy the blocked story, mark it passing.
7. If you cannot fully satisfy it, you must still improve the batch:
   a. add reducers, prerequisites, or a narrower next gate
   b. update the PRD file
   c. update the blocked story notes with the new evidence
   d. append a short summary to `progress.txt`
8. Only work on this blocker and its immediate prerequisite decomposition.
   Do not drift into unrelated later stories.
9. Commit your changes with a message referencing the blocked story ID or the new
   prerequisite story ID you introduced.

## Important
- Do not keep rerunning the same broad gate if it is not producing a new reducer
  or narrower next step.
- If the blocker is too broad, rewrite the PRD around it so the next iteration
  has smaller, deterministic work.
- Use your rust skill.
- Use subagents whenever possible.
- Any new markdown documentation goes to aidocs/ on the project root. Do not commit those.
"""


def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


def load_prd(path: Path):
    return json.loads(path.read_text())


def prd_signature(prd_file: Path, prd: dict) -> dict:
    return {
        "prd_file": prd_file.name,
        "id": prd.get("id"),
        "title": prd.get("title"),
    }


def prd_stories(prd: dict) -> list[dict]:
    stories = prd.get("stories")
    if isinstance(stories, list):
        return stories
    stories = prd.get("userStories")
    if isinstance(stories, list):
        return stories
    return []


def all_stories_pass(prd: dict) -> bool:
    stories = prd_stories(prd)
    return bool(stories) and all(bool(s.get("passes")) for s in stories)


def prd_pass_states(prd: dict):
    return [bool(story.get("passes", False)) for story in prd_stories(prd)]


def first_incomplete_story(prd: dict):
    for story in prd_stories(prd):
        if not story.get("passes", False):
            return story
    return None


def first_actionable_story(prd: dict):
    stories = prd_stories(prd)
    story_map = {s.get("id"): s for s in stories}
    for story in stories:
        if story.get("passes", False):
            continue
        if story.get("status") == "blocked":
            continue
        deps = story.get("dependsOn", [])
        if all(story_map.get(dep, {}).get("passes", False) for dep in deps):
            return story
    return None


def story_fingerprint(story: dict | None):
    if story is None:
        return None
    return {
        "id": story.get("id"),
        "passes": story.get("passes"),
        "status": story.get("status"),
        "notes": story.get("notes"),
        "dependsOn": story.get("dependsOn", []),
        "title": story.get("title"),
    }


def opencode_settings():
    opencode_bin = os.environ.get("OPENCODE_BIN") or shutil.which("opencode")
    if not opencode_bin:
        eprint("Error: opencode not found on PATH. Set OPENCODE_BIN=/path/to/opencode.")
        sys.exit(1)
    return opencode_bin


def build_opencode_args(cwd: Path, continue_mode: bool = False, model: str | None = None):
    opencode_bin = opencode_settings()
    args = [opencode_bin, "run", "--format", "json"]
    if continue_mode:
        args.append("--continue")
    if model:
        args.extend(["--model", model])
    args.extend(["--dir", str(cwd)])
    return args


class Runner:
    def __init__(
        self,
        prd_file: Path,
        progress_file: Path,
        max_iterations: int,
        blocked_retry_limit: int,
        continue_mode: bool,
        session_meta_file: Path,
        retry_state_file: Path,
    ):
        self.prd_file = prd_file
        self.progress_file = progress_file
        self.max_iterations = max_iterations
        self.blocked_retry_limit = blocked_retry_limit
        self.continue_mode = continue_mode
        self.session_meta_file = session_meta_file
        self.retry_state_file = retry_state_file
        self.current_proc = None
        self.blocked_retries: dict[str, int] = {}
        self.session_id: str | None = None

    def ensure_files(self):
        if not self.prd_file.is_file():
            eprint(f"Error: {self.prd_file} not found.")
            sys.exit(1)
        if not self.progress_file.exists():
            self.progress_file.write_text("# Ralph Progress Log\n\n")

    def handle_signal(self, signum, _frame):
        if self.current_proc and self.current_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.current_proc.pid), signum)
            except ProcessLookupError:
                pass
            except Exception:
                try:
                    self.current_proc.send_signal(signum)
                except Exception:
                    pass
            try:
                self.current_proc.wait(timeout=5)
            except Exception:
                pass
        raise SystemExit(130)

    def prompt_for(self):
        return PROMPT_TEMPLATE

    def blocker_prompt_for(self, story: dict):
        return (
            BLOCKER_PROMPT_TEMPLATE.replace("__STORY_ID__", str(story.get("id", "")))
            .replace("__STORY_TITLE__", str(story.get("title", "")))
        )

    def continue_prompt_for(self, story: dict):
        description = (
            story.get("description")
            or story.get("summary")
            or story.get("acceptanceCriteria")
        )
        if isinstance(description, list):
            description = " ".join(str(item) for item in description)
        description_segment = (
            f" Selected story description: {description}." if description else ""
        )
        return (
            CONTINUE_PROMPT_TEMPLATE.replace("__STORY_ID__", str(story.get("id", "")))
            .replace("__STORY_TITLE__", str(story.get("title", "")))
            .replace("__STORY_DESCRIPTION_SEGMENT__", description_segment)
        )

    def load_session_meta(self):
        if not self.session_meta_file.exists():
            return None
        try:
            return json.loads(self.session_meta_file.read_text())
        except Exception:
            return None

    def save_session_meta(self, prd: dict):
        self.session_meta_file.write_text(
            json.dumps(prd_signature(self.prd_file, prd), indent=2) + "\n"
        )

    def clear_session_meta(self):
        try:
            self.session_meta_file.unlink()
        except FileNotFoundError:
            pass

    def load_retry_state(self):
        if not self.retry_state_file.exists():
            return None
        try:
            return json.loads(self.retry_state_file.read_text())
        except Exception:
            return None

    def save_retry_state(self, story_id: str, pass_states: list[bool], count: int):
        self.retry_state_file.write_text(
            json.dumps(
                {
                    "story_id": story_id,
                    "pass_states": pass_states,
                    "count": count,
                },
                indent=2,
            )
            + "\n"
        )

    def clear_retry_state(self):
        try:
            self.retry_state_file.unlink()
        except FileNotFoundError:
            pass

    def mark_story_blocked(self, story_id: str, note: str):
        prd = load_prd(self.prd_file)
        changed = False
        for story in prd_stories(prd):
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

    def should_stop_on_blocked(self, story: dict) -> bool:
        if story.get("status") != "blocked":
            return False
        story_id = story.get("id", "")
        count = self.blocked_retries.get(story_id, 0) + 1
        self.blocked_retries[story_id] = count
        if count > self.blocked_retry_limit:
            print(f"Blocked story {story_id} exceeded retry limit ({self.blocked_retry_limit}). Stopping.")
            return True
        print(f"Blocked story {story_id} retry {count}/{self.blocked_retry_limit}.")
        return False

    def run_opencode(self, prompt: str, continue_mode: bool) -> int:
        cwd = Path.cwd()
        args = build_opencode_args(cwd, continue_mode=continue_mode)
        self.session_id = None

        self.current_proc = subprocess.Popen(
            args,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            text=True,
            bufsize=1,
        )
        assert self.current_proc.stdin is not None
        assert self.current_proc.stdout is not None

        self.current_proc.stdin.write(prompt)
        self.current_proc.stdin.close()

        step_finished = False
        for line in self.current_proc.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            if line.startswith("{"):
                try:
                    event = json.loads(line)
                    if (
                        event.get("type") == "step_finish"
                        and event.get("part", {}).get("reason") == "stop"
                    ):
                        step_finished = True
                    if self.session_id is None:
                        for key in ("session_id", "sessionId", "id"):
                            if key in event:
                                self.session_id = str(event[key])
                                break
                except json.JSONDecodeError:
                    pass

        rc = self.current_proc.wait()
        self.current_proc = None

        if not continue_mode and self.session_id:
            opencode_bin = opencode_settings()
            try:
                subprocess.run(
                    [opencode_bin, "session", "delete", self.session_id],
                    check=False,
                    capture_output=True,
                )
            except Exception:
                pass
            self.session_id = None

        return rc, step_finished

    def run(self):
        self.ensure_files()

        print("=== Ralph Loop (OpenCode) ===")
        print(f"PRD: {self.prd_file.name}")
        print(f"Session mode: {'continuable' if self.continue_mode else 'ephemeral'}")
        print(f"Blocked retry limit: {self.blocked_retry_limit}")
        print(f"Continue mode: {'on' if self.continue_mode else 'off'}")
        print()

        signal.signal(signal.SIGINT, self.handle_signal)
        signal.signal(signal.SIGTERM, self.handle_signal)

        first_iteration = True

        for iteration in range(1, self.max_iterations + 1):
            prd = load_prd(self.prd_file)
            current_prd_sig = prd_signature(self.prd_file, prd)
            pre_attempt_pass_states = prd_pass_states(prd)
            if all_stories_pass(prd):
                print("All stories complete!")
                return 0

            blocked_attempt = False
            story = first_actionable_story(prd)
            if story is None:
                blocked_story = first_incomplete_story(prd)
                if blocked_story is not None:
                    if self.should_stop_on_blocked(blocked_story):
                        return 2
                    print(
                        "No actionable incomplete stories remain in this iteration. "
                        f"First incomplete story is {blocked_story.get('id')} "
                        f"with status={blocked_story.get('status', 'pending')}. "
                        "Attempting to solve the blocker directly."
                    )
                    print()
                    story = blocked_story
                    blocked_attempt = True
                    blocked_story_before = story_fingerprint(blocked_story)
                else:
                    print("All stories complete!")
                    return 0
            else:
                blocked_story_before = None

            next_story = f"{story.get('id')}: {story.get('title')}"
            now = subprocess.check_output(["date", "+%H:%M:%S"], text=True).strip()
            print(f"--- Iteration {iteration} — {now} — next story: {next_story} ---")

            if self.continue_mode:
                saved_meta = self.load_session_meta()
                if saved_meta is not None and saved_meta != current_prd_sig:
                    print(
                        "Stored session metadata belongs to a different PRD. "
                        "Clearing session metadata and starting this batch fresh."
                    )
                    self.clear_session_meta()
                    first_iteration = True

            if blocked_attempt:
                prompt = self.blocker_prompt_for(story)
                use_continue = self.continue_mode
            else:
                prompt = (
                    self.continue_prompt_for(story)
                    if (self.continue_mode and not first_iteration)
                    else self.prompt_for()
                )
                use_continue = self.continue_mode and not first_iteration

            rc, step_finished = self.run_opencode(prompt, continue_mode=use_continue)

            if rc != 0 and not step_finished:
                print(f"OpenCode exited with status {rc} and did not reach step_finish.")
                return rc

            first_iteration = False

            if self.continue_mode:
                self.save_session_meta(prd)

            prd_after = load_prd(self.prd_file)
            post_attempt_pass_states = prd_pass_states(prd_after)
            story_id = story.get("id")
            story_after = next(
                (s for s in prd_stories(prd_after) if s.get("id") == story_id),
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
                        f"Auto-blocked by ralph_opencode.py on {subprocess.check_output(['date', '+%F'], text=True).strip()}: "
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
                blocked_story_after = first_incomplete_story(prd_after)
                if story_fingerprint(blocked_story_after) != blocked_story_before:
                    story_id = blocked_story_before.get("id")
                    if story_id in self.blocked_retries:
                        self.blocked_retries[story_id] = 0
                    print(
                        f"Blocked story {story_id} changed after this attempt. "
                        "Resetting blocked retry counter."
                    )

            print()
            print(f"Iteration {iteration} complete.")
            print()

        print(f"Reached max iterations ({self.max_iterations}). Stopping.")
        return 1


def main(argv: list[str]) -> int:
    args = list(argv[1:])
    continue_mode = False
    if "--continue" in args:
        args.remove("--continue")
        continue_mode = True
    prd_arg = args[0] if args else "prd.json"
    runner = Runner(
        prd_file=Path(prd_arg),
        progress_file=Path("progress.txt"),
        max_iterations=int(os.environ.get("MAX_ITERATIONS", "300")),
        blocked_retry_limit=int(os.environ.get("BLOCKED_RETRY_LIMIT", "3")),
        continue_mode=continue_mode,
        session_meta_file=Path(".session_meta.json"),
        retry_state_file=Path(".ralph_retry_state.json"),
    )
    return runner.run()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
