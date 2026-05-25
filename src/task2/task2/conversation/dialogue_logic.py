from __future__ import annotations

from collections import Counter
from collections.abc import Callable

from task2.conversation.task_parser import (
    TASK_CONFIRMATION_SPEECH,
    TASK_SPEECH,
    is_affirmative,
    parse_task,
)


class DialogueSession:
    def __init__(
        self,
        gender: str,
        speak: Callable[[str], None],
        listen: Callable[[], str],
        log: Callable[[str], None] | None = None,
    ) -> None:
        self.gender = gender
        self._speak = speak
        self._listen = listen
        self._log = log or (lambda _msg: None)

    def run(self) -> str:
        if self.gender == "female":
            return self._run_female()
        return self._run_male()

    def _prompt_for_task(self) -> None:
        person_word = "woman" if self.gender == "female" else "man"
        self._say(f"Hi {person_word}, which task should I perform?")

    def _run_male(self) -> str:
        self._prompt_for_task()
        while True:
            utterance = self._listen()
            result = parse_task(utterance)
            self._log_parse(utterance, result.task, result.ambiguous)
            if result.ambiguous:
                self._say("Which cell should I inspect, red or green?")
                continue
            if result.task is None:
                self._say("Can you repeat yourself please?")
                continue
            self._say_final(result.task)
            return result.task

    def _run_female(self) -> str:
        task_counts: Counter[str] = Counter()
        current_task: str | None = None
        first_confirmation = True

        self._prompt_for_task()
        while True:
            utterance = self._listen()
            result = parse_task(utterance)
            self._log_parse(utterance, result.task, result.ambiguous)
            if result.ambiguous:
                self._say("Which cell should I inspect, red or green?")
                continue
            if result.task is None:
                self._say("Can you repeat yourself please?")
                continue

            current_task = result.task
            task_counts[current_task] += 1
            if task_counts[current_task] >= 2:
                self._log(f"auto-accepted repeated female task: {current_task}")
                self._say_final(current_task)
                return current_task

            if first_confirmation:
                self._say("Are you sure?")
                first_confirmation = False
            else:
                short_task = TASK_CONFIRMATION_SPEECH[current_task]
                self._say(f"OK, the {short_task} then. Are you sure?")

            while True:
                confirmation = self._listen()
                if is_affirmative(confirmation):
                    self._log(f"confirmation accepted: {confirmation}")
                    self._say_final(current_task)
                    return current_task

                next_result = parse_task(confirmation)
                self._log_parse(confirmation, next_result.task, next_result.ambiguous)
                if next_result.ambiguous:
                    self._say("Which cell should I inspect, red or green?")
                    break
                if next_result.task is None:
                    self._say("Can you repeat yourself please?")
                    break

                current_task = next_result.task
                task_counts[current_task] += 1
                if task_counts[current_task] >= 2:
                    self._log(f"auto-accepted repeated female task: {current_task}")
                    self._say_final(current_task)
                    return current_task

                short_task = TASK_CONFIRMATION_SPEECH[current_task]
                self._say(f"OK, the {short_task} then. Are you sure?")

    def _say(self, text: str) -> None:
        self._log(f"robot prompt: {text}")
        self._speak(text)

    def _say_final(self, task: str) -> None:
        self._say(f"OK. I will {TASK_SPEECH[task]}.")

    def _log_parse(self, utterance: str, task: str | None, ambiguous: bool) -> None:
        self._log(
            f"user utterance: {utterance!r}, task={task or 'none'}, ambiguous={ambiguous}"
        )
