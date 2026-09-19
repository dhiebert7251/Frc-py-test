package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.FireCommand;
import frc.robot.subsystems.Trigger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for Trigger's FireCommand edge-detection state machine -- a Java
 * translation of the Python sibling's {@code test_trigger.py}.
 *
 * <p>The cam has only one sensor (a limit switch at "home"), so "one fire" is defined
 * as: leave home, then come back to home. These tests exercise that logic directly
 * against the DigitalInput simulation, without needing a real cam mechanism.
 * FireCommand itself has no built-in timeout (see its docstring) -- the second test
 * below applies the same {@code .withTimeout()} decorator RobotContainer.java binds it
 * with, to prove the safety timeout actually works.
 */
class TriggerTest {

    private RobotContainer robotContainer;
    private Trigger trigger;

    @BeforeEach
    void setup() {
        if (!HAL.initialize(500, 0)) {
            throw new IllegalStateException("HAL failed to initialize");
        }
        robotContainer = new RobotContainer();
        trigger = robotContainer.trigger;
    }

    @AfterEach
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        HAL.shutdown();
    }

    private void step(double seconds) {
        SimHooks.stepTiming(seconds);
        CommandScheduler.getInstance().run();
    }

    @Test
    void fireCommandFinishesWhenCamReturnsHome() {
        DIOSim limitSwitchSim = new DIOSim(Constants.TriggerConstants.LIMIT_SWITCH_DIO_PORT);

        // Start "at home" (limit switch reads true, not inverted).
        limitSwitchSim.setValue(true);
        assertTrue(trigger.isAtHome());

        // A command can only be scheduled while the robot is enabled (the default
        // runsWhenDisabled() is false), so enable it first.
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        step(0.02);

        FireCommand command = new FireCommand(trigger);
        command.schedule();
        step(0.1);
        // Still "at home" on the very first tick -- command must not report finished
        // until it has actually left home at least once.
        assertTrue(command.isScheduled());

        // Simulate the cam leaving home.
        limitSwitchSim.setValue(false);
        step(0.1);
        assertTrue(command.isScheduled());

        // Simulate the cam returning home -- command should finish now.
        limitSwitchSim.setValue(true);
        step(0.1);
        assertFalse(command.isScheduled());
    }

    @Test
    void fireCommandTimesOutIfNeverReturnsHome() {
        DIOSim limitSwitchSim = new DIOSim(Constants.TriggerConstants.LIMIT_SWITCH_DIO_PORT);

        limitSwitchSim.setValue(false); // never at home -- simulates a jam
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        step(0.02);

        // .withTimeout() is the decorator RobotContainer.java actually binds
        // FireCommand with -- applying it here too is what proves the timeout (not
        // just the edge-detection logic) really stops a jammed cam.
        //
        // `.withTimeout(...)` does NOT modify the FireCommand it's called on -- it
        // wraps it in a brand new Command object that composes the original
        // internally. That wrapper is what actually gets scheduled, so it's the
        // wrapper's `isScheduled()` this test has to check below, captured here in a
        // variable typed as the general `Command` interface rather than `FireCommand`
        // (the wrapper is not itself a FireCommand). Checking the original
        // `FireCommand` object's own `isScheduled()` instead would be a real mistake:
        // the scheduler only ever registers the outer wrapper, so the inner
        // FireCommand's `isScheduled()` would incorrectly read `false` from the very
        // first loop, making this test pass without actually exercising the timeout.
        Command timedCommand = new FireCommand(trigger).withTimeout(Constants.TriggerConstants.FIRE_TIMEOUT_SECONDS);
        timedCommand.schedule();
        step(Constants.TriggerConstants.FIRE_TIMEOUT_SECONDS + 0.5);
        assertFalse(timedCommand.isScheduled());
    }
}
