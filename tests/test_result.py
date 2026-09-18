"""Pure unit tests for result.py -- no hardware/HAL involved."""

from result import Result


def test_passed():
    r = Result.passed("drivetrain", "wheels spin")
    assert r.is_success()
    assert not r.is_failure()
    assert r.unit == "drivetrain"
    assert r.reports == ["wheels spin"]


def test_failed():
    r = Result.failed("shooter", "did not reach target RPM")
    assert r.is_failure()
    assert not r.is_success()


def test_str_contains_unit_and_reports():
    r = Result.passed("vision", "front camera connected")
    text = str(r)
    assert "vision" in text
    assert "front camera connected" in text
