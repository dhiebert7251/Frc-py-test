"""Smoke test: step the whole robot through disabled -> autonomous -> teleop
and confirm nothing raises. This is an automated version of the manual
verification this port was checked with before it was first pushed.
"""


def test_full_mode_cycle(control):
    with control.run_robot():
        control.step_timing(seconds=0.1, autonomous=False, enabled=False)
        control.step_timing(seconds=1.0, autonomous=True, enabled=True)
        control.step_timing(seconds=0.1, autonomous=False, enabled=False)
        control.step_timing(seconds=1.0, autonomous=False, enabled=True)
        control.step_timing(seconds=0.1, autonomous=False, enabled=False)
