package frc.robot.subsystems;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class AprilTags {
    private AprilTags() {}

    // Welded Perimeter coordinates from the provided table (inches, Z-rotation degrees).
    private static final Map<Integer, Pose3d> TAG_POSES = Map.ofEntries(
        Map.entry(1, poseInches(467.64, 292.31, 35.00, 180)),
        Map.entry(2, poseInches(469.11, 182.60, 44.25, 90)),
        Map.entry(3, poseInches(445.35, 172.84, 44.25, 180)),
        Map.entry(4, poseInches(445.35, 158.84, 44.25, 180)),
        Map.entry(5, poseInches(469.11, 135.09, 44.25, 270)),
        Map.entry(6, poseInches(467.64, 25.37, 35.00, 180)),
        Map.entry(7, poseInches(470.59, 25.37, 35.00, 0)),
        Map.entry(8, poseInches(483.11, 135.09, 44.25, 270)),
        Map.entry(9, poseInches(492.88, 144.84, 44.25, 0)),
        Map.entry(10, poseInches(492.88, 158.84, 44.25, 0)),
        Map.entry(11, poseInches(483.11, 182.60, 44.25, 90)),
        Map.entry(12, poseInches(470.59, 292.31, 35.00, 0)),
        Map.entry(13, poseInches(650.92, 291.47, 21.75, 180)),
        Map.entry(14, poseInches(650.92, 274.47, 21.75, 180)),
        Map.entry(15, poseInches(650.90, 170.22, 21.75, 180)),
        Map.entry(16, poseInches(650.90, 153.22, 21.75, 180)),
        Map.entry(17, poseInches(183.59, 25.37, 35.00, 0)),
        Map.entry(18, poseInches(182.11, 135.09, 44.25, 270)),
        Map.entry(19, poseInches(205.87, 144.84, 44.25, 0)),
        Map.entry(20, poseInches(205.87, 158.84, 44.25, 0)),
        Map.entry(21, poseInches(182.11, 182.60, 44.25, 90)),
        Map.entry(22, poseInches(183.59, 292.31, 35.00, 0)),
        Map.entry(23, poseInches(180.64, 292.31, 35.00, 180)),
        Map.entry(24, poseInches(168.11, 182.60, 44.25, 90)),
        Map.entry(25, poseInches(158.34, 172.84, 44.25, 180)),
        Map.entry(26, poseInches(158.34, 158.84, 44.25, 180)),
        Map.entry(27, poseInches(168.11, 135.09, 44.25, 270)),
        Map.entry(28, poseInches(180.64, 25.37, 35.00, 180)),
        Map.entry(29, poseInches(0.30, 26.22, 21.75, 0)),
        Map.entry(30, poseInches(0.30, 43.22, 21.75, 0)),
        Map.entry(31, poseInches(0.32, 147.47, 21.75, 0)),
        Map.entry(32, poseInches(0.32, 164.47, 21.75, 0))
    );

    public static Optional<Pose3d> getPose(int id) {
        return Optional.ofNullable(TAG_POSES.get(id));
    }

    public static Map<Integer, Pose3d> getAllPoses() {
        return TAG_POSES;
    }

    private static Pose3d poseInches(double xIn, double yIn, double zIn, double zRotDeg) {
        return new Pose3d(
            new Translation3d(
                Units.inchesToMeters(xIn),
                Units.inchesToMeters(yIn),
                Units.inchesToMeters(zIn)
            ),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(zRotDeg))
        );
    }
}
