"""Result container for the (future) component test mode.

Ported from Result.java. Note: Java's static factory methods `pass(...)` and
`fail(...)` are renamed to `passed(...)` / `failed(...)` here since `pass` is
a reserved keyword in Python.
"""

from dataclasses import dataclass, field


@dataclass
class Result:
    unit: str
    success: bool
    reports: list[str] = field(default_factory=list)

    @staticmethod
    def passed(unit: str, *reports: str) -> "Result":
        return Result(unit, True, list(reports))

    @staticmethod
    def failed(unit: str, *reports: str) -> "Result":
        return Result(unit, False, list(reports))

    def is_success(self) -> bool:
        return self.success

    def is_failure(self) -> bool:
        return not self.success

    def __str__(self) -> str:
        reset = "\033[0m"
        green = "\033[32m"
        red = "\033[31m"
        color = green if self.success else red
        status = "Success!" if self.success else "Failure!"
        header = f">> {color} {status} {reset}: {self.unit}\n"
        body = "".join(f">>\t{report}\n\n" for report in self.reports)
        return header + body
