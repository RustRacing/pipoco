#!/usr/bin/env python3
import json
import os
import re
import shutil
import signal
import subprocess
import sys
from pathlib import Path

PROMPT_TEMPLATE = r'''You are Ralph, an autonomous coding agent. You execute user stories from __PRD_FILE__ one at a time.

## Instructions

1. Read `__PRD_FILE__` to understand the project and find the next incomplete story (first story where `"passes": false`, in priority order).
2. If all stories have `"passes": true`, say "All stories complete." and exit.
3. For the chosen story:
   a. Read its description and acceptance criteria carefully.
   b. Implement the changes needed to satisfy ALL acceptance criteria.
   c. After implementing, verify each acceptance criterion is met (run compiler checks, lints, and tests as specified).
   d. If all criteria pass, update `__PRD_FILE__` — set that story's `"passes": true` and add any relevant notes.
   e. Append a short summary of what you did to `progress.txt`.
4. Only work on ONE story per iteration. After completing (or failing) one story, stop.
5. If you cannot complete a story, set its `"notes"` field with what went wrong and stop. Do NOT mark it as passing.
6. Commit your changes with a message referencing the story ID (e.g., "US-001: Add status column to tasks table").

## Important
- You have no memory of previous iterations. Read __PRD_FILE__ and progress.txt to understand current state.
- You have a lot of time. Don't call the acceptance criteria with everything broken or with things you can fix before submitting.
- Since a task can be long, do not give up on compiler errors, use your $rust skill to solve.
- Use the context mode mcp
- Large refactors are allowed, but plan them well.
- Do not skip acceptance criteria. Every criterion must be verified.
- Do not work on more than one story.
- After finishing the story (pass or fail), stop immediately.
- any new markdown should go to aidocs/ on root folder of the project. Do not commit those
'''

CONTINUE_PROMPT_TEMPLATE = (
    "Continue this batch. Read the PRD and progress log again, then work only on this already-selected "
    "pending actionable story: __STORY_ID__ — __STORY_TITLE__.__STORY_DESCRIPTION_SEGMENT__ Do not pick "
    "a different story and do not pick a blocked story under this prompt; blocked stories are handled only "
    "by the blocker prompt path. If this pending task is not making progress, mark it as blocked for "
    "replanning. Otherwise complete exactly this one story and stop. Remember to use $rust, context mcp "
    "and subagents with gpt-5.4-mini low model (use subagents whenever possible as it saves credits)."
)

BLOCKER_PROMPT_TEMPLATE = r'''You are Ralph, an autonomous coding agent. You are handling a blocked story from __PRD_FILE__.

Blocked story: __STORY_ID__ — __STORY_TITLE__

## Instructions

1. Read `__PRD_FILE__` and `progress.txt`.
2. Focus on the first incomplete story, which is currently blocked. However, consider fixing from tasks too as giving up is way worse.
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
   b. update `__PRD_FILE__`
   c. update the blocked story notes with the new evidence
   d. append a short summary to `progress.txt`
8. Only work on this blocker and its immediate prerequisite decomposition. Do not drift into unrelated later stories.
9. Commit your changes with a message referencing the blocked story ID or the new prerequisite story ID you introduced.

## Important
- Do not keep rerunning the same broad gate if it is not producing a new reducer or narrower next step.
- If the blocker is too broad, rewrite the PRD around it so the next iteration has smaller, deterministic work.
- Use your $rust skill.
- Use the context mode mcp.
- any new markdown should go to aidocs/ on root folder of the project. Do not commit those
'''

SESSION_ID_PATTERN = re.compile(r"session id:\s*([0-9a-fA-F-]{36})")
RESUME_INVALID_PATTERNS = (
    "thread/resume failed: no rollout found",
    "no rollout found for thread id",
)


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


def prd_context_file(prd_file: Path, prd: dict) -> Path | None:
    for key in ("contextFile", "context_file", "bootstrapFile", "bootstrap_file"):
        value = prd.get(key)
        if isinstance(value, str) and value.strip():
            path = Path(value.strip())
            if not path.is_absolute():
                path = prd_file.parent / path
            return path
    return None


def display_path(path: Path) -> str:
    try:
        return os.path.relpath(path.resolve(), Path.cwd().resolve())
    except Exception:
        return str(path)


def parse_runner_cli(argv: list[str], default_prd: str):
    args = list(argv[1:])
    continue_mode = False
    context_file = None
    prd_arg = None
    idx = 0
    while idx < len(args):
        arg = args[idx]
        if arg == "--continue":
            continue_mode = True
            idx += 1
            continue
        if arg == "--context-file":
            if idx + 1 >= len(args):
                eprint("Error: --context-file requires a path.")
                raise SystemExit(2)
            context_file = args[idx + 1]
            idx += 2
            continue
        if arg.startswith("--context-file="):
            context_file = arg.split("=", 1)[1]
            idx += 1
            continue
        if arg.startswith("-"):
            eprint(f"Error: unknown option {arg}")
            raise SystemExit(2)
        if prd_arg is None:
            prd_arg = arg
            idx += 1
            continue
        eprint(f"Error: unexpected extra argument {arg}")
        raise SystemExit(2)
    prd_path = Path(prd_arg or default_prd)
    resolved_context = None
    if context_file:
        resolved_context = Path(context_file)
        if not resolved_context.is_absolute():
            resolved_context = prd_path.parent / resolved_context
    return prd_path, continue_mode, resolved_context


def codex_settings():
    codex_bin = os.environ.get("CODEX_BIN") or shutil.which("codex")
    if not codex_bin:
        eprint("Error: codex not found on PATH. Set CODEX_BIN=/path/to/codex.")
        sys.exit(1)
    model = os.environ.get("CODEX_MODEL", "gpt-5.3-codex")
    reasoning = os.environ.get("CODEX_REASONING", "low")
    bypass = os.environ.get("CODEX_BYPASS_SANDBOX", "1") == "1"
    sandbox = os.environ.get("CODEX_SANDBOX", "workspace-write")

    return codex_bin, model, reasoning, bypass, sandbox


def prune_command():
    return os.environ.get("RALPH_PRUNE_CMD", "make prune")


def build_codex_args(cwd: Path, reasoning_override: str | None = None):
    codex_bin, model, reasoning, bypass, sandbox = codex_settings()
    args = [
        codex_bin,
        "exec",
        "-C",
        str(cwd),
        "--model",
        model,
        "--config",
        f'model_reasoning_effort="{reasoning_override or reasoning}"',
    ]
    if bypass:
        args.append("--dangerously-bypass-approvals-and-sandbox")
    else:
        args.extend(["--sandbox", sandbox])
    args.append("-")
    return args, model, reasoning


def build_codex_resume_args(session_id: str, reasoning_override: str | None = None):
    codex_bin, model, reasoning, bypass, _sandbox = codex_settings()
    args = [
        codex_bin,
        "exec",
        "resume",
        "--model",
        model,
        "--config",
        f'model_reasoning_effort="{reasoning_override or reasoning}"',
    ]
    if bypass:
        args.append("--dangerously-bypass-approvals-and-sandbox")
    args.extend([session_id, "-"])
    return args


class Runner:
    def __init__(
        self,
        prd_file: Path,
        progress_file: Path,
        max_iterations: int,
        blocked_retry_limit: int,
        continue_mode: bool,
        session_file: Path,
        session_meta_file: Path,
        retry_state_file: Path,
        context_file: Path | None,
    ):
        self.prd_file = prd_file
        self.progress_file = progress_file
        self.max_iterations = max_iterations
        self.blocked_retry_limit = blocked_retry_limit
        self.continue_mode = continue_mode
        self.session_file = session_file
        self.session_meta_file = session_meta_file
        self.retry_state_file = retry_state_file
        self.context_file = context_file
        self.current_proc = None
        self.blocked_retries: dict[str, int] = {}

    def ensure_files(self):
        if not self.prd_file.is_file():
            eprint(f"Error: {self.prd_file} not found.")
            sys.exit(1)
        prd = load_prd(self.prd_file)
        context_file = self.resolved_context_file(prd)
        if context_file is not None and not context_file.is_file():
            eprint(f"Error: context file {display_path(context_file)} not found.")
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

    def run_prune(self):
        cmd = prune_command().strip()
        if not cmd:
            return
        print(f"--- Post-iteration prune: {cmd}")
        try:
            result = subprocess.run(
                ["bash", "-lc", cmd],
                text=True,
                capture_output=True,
                check=False,
            )
        except Exception as exc:
            print(f"Prune command failed to start: {exc}")
            return
        output = (result.stdout or "") + (result.stderr or "")
        if output.strip():
            print(output.rstrip())
        if result.returncode != 0:
            print(f"Prune command exited with status {result.returncode}. Continuing.")

    def resolved_context_file(self, prd: dict | None = None) -> Path | None:
        if self.context_file is not None:
            return self.context_file
        if prd is None:
            prd = load_prd(self.prd_file)
        return prd_context_file(self.prd_file, prd)

    def with_context_prompt(self, prompt: str, prd: dict | None = None):
        context_file = self.resolved_context_file(prd)
        if context_file is None:
            return prompt
        context_path = display_path(context_file)
        return (
            prompt.rstrip()
            + "\n\n## Bootstrap Context\n"
            + f"- Read `{context_path}` before choosing edits.\n"
            + "- Treat it as durable batch context alongside the PRD and progress log.\n"
        )

    def prompt_for(self):
        return self.with_context_prompt(
            PROMPT_TEMPLATE.replace("__PRD_FILE__", self.prd_file.name)
        )

    def blocker_prompt_for(self, story: dict):
        return self.with_context_prompt(
            BLOCKER_PROMPT_TEMPLATE.replace("__PRD_FILE__", self.prd_file.name)
            .replace("__STORY_ID__", str(story.get("id", "")))
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
        return self.with_context_prompt(
            CONTINUE_PROMPT_TEMPLATE.replace("__STORY_ID__", str(story.get("id", "")))
            .replace("__STORY_TITLE__", str(story.get("title", "")))
            .replace("__STORY_DESCRIPTION_SEGMENT__", description_segment)
        )

    def load_session_id(self):
        if not self.session_file.exists():
            return None
        session_id = self.session_file.read_text().strip()
        return session_id or None

    def save_session_id(self, session_id: str):
        self.session_file.write_text(session_id + "\n")

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

    def clear_session_id(self):
        try:
            self.session_file.unlink()
        except FileNotFoundError:
            pass
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

    def run(self):
        self.ensure_files()
        args, model, reasoning = build_codex_args(Path.cwd())

        print("=== Ralph Loop ===")
        print(f"PRD: {self.prd_file.name}")
        print(f"Model: {model}")
        print(f"Reasoning: {reasoning}")
        print(f"Session mode: {'continuable' if self.continue_mode else 'ephemeral'}")
        print(f"Blocked retry limit: {self.blocked_retry_limit}")
        print(f"Continue mode: {'on' if self.continue_mode else 'off'}")
        context_file = self.resolved_context_file(load_prd(self.prd_file))
        print(f"Context file: {display_path(context_file) if context_file else '(none)'}")
        print()

        signal.signal(signal.SIGINT, self.handle_signal)
        signal.signal(signal.SIGTERM, self.handle_signal)

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

            session_id = self.load_session_id() if self.continue_mode else None
            using_blocked_session_override = False
            if self.continue_mode and session_id:
                saved_meta = self.load_session_meta()
                if saved_meta != current_prd_sig:
                    print(
                        "Stored continuation session belongs to a different PRD. "
                        "Clearing .session_id and starting this batch fresh."
                    )
                    self.clear_session_id()
                    session_id = None
            blocked_session_id = os.environ.get("RALPH_BLOCKED_SESSION_ID", "").strip() or None
            if self.continue_mode and blocked_attempt and blocked_session_id:
                print(f"Blocked attempt using override session {blocked_session_id}.")
                session_id = blocked_session_id
                using_blocked_session_override = True
                attempted_resume = bool(self.continue_mode and session_id)
            else:
                attempted_resume = False  # resume makes the agent give up
            while True:
                blocked_reasoning = os.environ.get("CODEX_BLOCKED_REASONING", "high")
                if attempted_resume:
                    iter_args = build_codex_resume_args(
                        session_id,
                        blocked_reasoning if blocked_attempt else None,
                    )
                    prompt = (
                        self.blocker_prompt_for(story)
                        if blocked_attempt
                        else self.continue_prompt_for(story)
                    )
                else:
                    iter_args = build_codex_args(
                        Path.cwd(),
                        blocked_reasoning if blocked_attempt else None,
                    )[0]
                    prompt = self.blocker_prompt_for(story) if blocked_attempt else self.prompt_for()

                self.current_proc = subprocess.Popen(
                    iter_args,
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
                seen_session_id = session_id if attempted_resume else None
                output_lines = []
                for line in self.current_proc.stdout:
                    output_lines.append(line)
                    sys.stdout.write(line)
                    sys.stdout.flush()
                    if self.continue_mode and not seen_session_id:
                        match = SESSION_ID_PATTERN.search(line)
                        if match:
                            seen_session_id = match.group(1)
                rc = self.current_proc.wait()
                self.current_proc = None

                combined_output = "".join(output_lines)
                if attempted_resume and rc != 0 and any(p in combined_output for p in RESUME_INVALID_PATTERNS):
                    print("Stored .session_id is invalid. Clearing it and retrying this iteration with a fresh prompt.")
                    if not using_blocked_session_override:
                        self.clear_session_id()
                    session_id = None
                    attempted_resume = False
                    continue
                break

            if rc != 0:
                print(f"Codex exited with status {rc}.")
                self.run_prune()
                print()
                return rc

            if self.continue_mode and seen_session_id and not using_blocked_session_override:
                self.save_session_id(seen_session_id)
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
                        f"Auto-blocked by ralph_simple.py on {subprocess.check_output(['date', '+%F'], text=True).strip()}: "
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
            self.run_prune()
            print()

        print(f"Reached max iterations ({self.max_iterations}). Stopping.")
        return 1


def main(argv: list[str]) -> int:
    prd_file, continue_mode, context_file = parse_runner_cli(
        argv,
        "prd_next_long_batch.json",
    )
    runner = Runner(
        prd_file=prd_file,
        progress_file=Path("progress.txt"),
        max_iterations=int(os.environ.get("MAX_ITERATIONS", "300")),
        blocked_retry_limit=int(os.environ.get("BLOCKED_RETRY_LIMIT", "6")),
        continue_mode=continue_mode,
        session_file=Path(".session_id"),
        session_meta_file=Path(".session_meta.json"),
        retry_state_file=Path(".ralph_retry_state.json"),
        context_file=context_file,
    )
    return runner.run()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
