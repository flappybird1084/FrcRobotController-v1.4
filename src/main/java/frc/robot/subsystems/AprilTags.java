package frc.robot.subsystems;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;

public final class AprilTags {
    private AprilTags() {}

    private static volatile int detectedCount;

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

    public static final class AprilTagMeasurement {
        public final int id;
        public final Translation3d translation;
        public final Rotation3d rotation;

        public AprilTagMeasurement(int id, double xMeters, double yMeters, double zMeters, Rotation3d rotation) {
            this.id = id;
            this.translation = new Translation3d(xMeters, yMeters, zMeters);
            this.rotation = rotation;
        }
    }

    public static Optional<Pose3d> getPose(int id) {
        return Optional.ofNullable(TAG_POSES.get(id));
    }

    public static Map<Integer, Pose3d> getAllPoses() {
        return TAG_POSES;
    }

    public static void setDetectedCount(int count) {
        detectedCount = Math.max(0, count);
    }

    public static int getDetectedCount() {
        return detectedCount;
    }

    public static Pose2d getRobotOffset(AprilTagMeasurement tagA, AprilTagMeasurement tagB) {
        return getRobotOffset(tagA, tagB, new Translation3d());
    }

    public static Pose2d getRobotOffset(
        AprilTagMeasurement tagA,
        AprilTagMeasurement tagB,
        Translation3d cameraToRobotOffset
    ) {
        Pose2d currentPose = RobotContainer.driveSubsystem.getPose();
        Optional<Pose3d> tagAPoseOpt = getPose(tagA.id);
        Optional<Pose3d> tagBPoseOpt = getPose(tagB.id);
        if (tagAPoseOpt.isEmpty() || tagBPoseOpt.isEmpty()) {
            return new Pose2d();
        }

        Pose3d tagAPose = tagAPoseOpt.get();
        Pose3d tagBPose = tagBPoseOpt.get();

        double fieldDx = tagBPose.getX() - tagAPose.getX();
        double fieldDy = tagBPose.getY() - tagAPose.getY();
        double obsDx = tagB.translation.getX() - tagA.translation.getX();
        double obsDy = tagB.translation.getY() - tagA.translation.getY();
        Rotation2d cameraYaw = new Rotation2d(Math.atan2(fieldDy, fieldDx) - Math.atan2(obsDy, obsDx));

        double distA = Math.hypot(tagA.translation.getX(), tagA.translation.getY());
        double distB = Math.hypot(tagB.translation.getX(), tagB.translation.getY());

        Pose2d cameraPose = triangulate2d(tagAPose, distA, tagBPose, distB, currentPose);
        Translation3d offsetField = new Translation3d(
            cameraToRobotOffset.getX() * cameraYaw.getCos() - cameraToRobotOffset.getY() * cameraYaw.getSin(),
            cameraToRobotOffset.getX() * cameraYaw.getSin() + cameraToRobotOffset.getY() * cameraYaw.getCos(),
            cameraToRobotOffset.getZ()
        );
        Pose2d actualRobotPose = new Pose2d(
            cameraPose.getX() + offsetField.getX(),
            cameraPose.getY() + offsetField.getY(),
            cameraYaw
        );

        return new Pose2d(
            actualRobotPose.getX() - currentPose.getX(),
            actualRobotPose.getY() - currentPose.getY(),
            actualRobotPose.getRotation().minus(currentPose.getRotation())
        );
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

    private static Pose2d triangulate2d(Pose3d tagA, double distA, Pose3d tagB, double distB, Pose2d currentPose) {
        double x0 = tagA.getX();
        double y0 = tagA.getY();
        double x1 = tagB.getX();
        double y1 = tagB.getY();
        double dx = x1 - x0;
        double dy = y1 - y0;
        double d = Math.hypot(dx, dy);
        if (d < 1e-6) {
            return new Pose2d(x0, y0, new Rotation2d());
        }

        double a = (distA * distA - distB * distB + d * d) / (2.0 * d);
        double hSq = distA * distA - a * a;
        if (hSq < 0.0) {
            hSq = 0.0;
        }
        double h = Math.sqrt(hSq);

        double xm = x0 + a * dx / d;
        double ym = y0 + a * dy / d;

        double rx = -dy * (h / d);
        double ry = dx * (h / d);

        Pose2d candidate1 = new Pose2d(xm + rx, ym + ry, new Rotation2d());
        Pose2d candidate2 = new Pose2d(xm - rx, ym - ry, new Rotation2d());

        double d1 = candidate1.getTranslation().getDistance(currentPose.getTranslation());
        double d2 = candidate2.getTranslation().getDistance(currentPose.getTranslation());
        return d1 <= d2 ? candidate1 : candidate2;
    }
}
