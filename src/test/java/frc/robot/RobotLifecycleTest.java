package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Smoke test: step the whole robot through disabled -&gt; autonomous -&gt; teleop and
 * confirm nothing throws. This is a Java translation of the Python sibling's
 * {@code test_robot_lifecycle.py}.
 *
 * <p>Python's version runs under pyfrc's pytest plugin, which provides ready-made
 * {@code robot}/{@code control} fixtures ({@code control.step_timing(...)} advances
 * simulated time AND flips the enabled/autonomous mode flags in one call). WPILib's
 * Java toolchain has no equivalent plugin, so this file does by hand what that fixture
 * did for free: {@link HAL#initialize} boots the simulated hardware layer,
 * {@link DriverStationSim} sets the enabled/autonomous mode flags a real driver station
 * would set, and {@link SimHooks#stepTiming} advances the simulated clock and lets
 * {@link CommandScheduler} run its periodic loop the corresponding number of times.
 * Every test class in this project's {@code src/test/} repeats this same
 * {@code @BeforeEach}/{@code @AfterEach} pair explicitly rather than hiding it behind a
 * shared base class -- consistent with this whole project's "explicit over implicit"
 * rule for anything a rookie might need to step through.
 */
class RobotLifecycleTest {

    private RobotContainer robotContainer;

    @BeforeEach
    void setup() {
        assertDoesNotThrow(() -> {
            if (!HAL.initialize(500, 0)) {
                throw new IllegalStateException("HAL failed to initialize");
            }
        });
        robotContainer = new RobotContainer();
    }

    @AfterEach
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        HAL.shutdown();
    }

    private void stepDisabled(double seconds) {
        DriverStationSim.setEnabled(false);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        SimHooks.stepTiming(seconds);
        CommandScheduler.getInstance().run();
    }

    private void stepAutonomous(double seconds) {
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
        SimHooks.stepTiming(seconds);
        CommandScheduler.getInstance().run();
    }

    private void stepTeleop(double seconds) {
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        SimHooks.stepTiming(seconds);
        CommandScheduler.getInstance().run();
    }

    @Test
    void fullModeCycleDoesNotThrow() {
        assertDoesNotThrow(() -> {
            stepDisabled(0.1);
            var autoCommand = robotContainer.getAutonomousCommand();
            if (autoCommand != null) {
                autoCommand.schedule();
            }
            stepAutonomous(1.0);
            stepDisabled(0.1);
            stepTeleop(1.0);
            stepDisabled(0.1);
        });
    }
}
