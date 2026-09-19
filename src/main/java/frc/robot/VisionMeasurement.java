package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Container for a single vision-based pose measurement from PhotonVision.
 *
 * <p>Teaching-bot proof of concept. Simpler than the competition bot's version: one
 * camera means there's no "which of two cameras' measurements do we trust more"
 * arbitration to carry alongside the data, so this only needs what DriveTrain's pose
 * estimator and ApproachTagCommand actually use.
 *
 * <p>{@code record} is a Java feature (since Java 16) purpose-built for exactly this:
 * a small, immutable bundle of named values with no behavior of its own. Writing
 * {@code public record VisionMeasurement(Pose2d estimatedPose, ...)} generates,
 * automatically, everything a hand-written class with the same fields would need --
 * a constructor, a getter for each field (named after the field itself, with no
 * {@code get} prefix -- {@code measurement.estimatedPose()}, not
 * {@code measurement.getEstimatedPose()}), and correct {@code equals()}/
 * {@code hashCode()}/{@code toString()} -- without writing any of that by hand. It's
 * the closest Java equivalent to Python's {@code @dataclass(frozen=True)}, used for
 * the exact same purpose on the Python sibling's own {@code VisionMeasurement}. This
 * project's real competition port ({@code 2026_competition_code}) uses this same
 * {@code record} pattern for its own {@code VisionMeasurement} class, accessed the
 * same parenthesized way ({@code measurement.estimatedPose()},
 * {@code measurement.numTagsUsed()}) -- that usage is where this file's shape was
 * confirmed from, rather than guessed.
 *
 * <p>{@code standardDeviations} is a {@code Matrix<N3, N1>} (a 3-row, 1-column
 * matrix) rather than a plain 3-tuple the way the Python sibling wrote it -- Java's
 * {@code DifferentialDrivePoseEstimator.addVisionMeasurement(...)} takes its standard
 * deviations as this typed matrix, built with {@code VecBuilder.fill(x, y, theta)};
 * RobotPy's Python binding for the same underlying method instead accepts a plain
 * tuple, which is why the two sibling projects' constants differ in shape here even
 * though they encode the exact same three numbers.
 */
public record VisionMeasurement(
    Pose2d estimatedPose,
    double timestampSeconds,
    Matrix<N3, N1> standardDeviations,
    int numTagsUsed
) {}
