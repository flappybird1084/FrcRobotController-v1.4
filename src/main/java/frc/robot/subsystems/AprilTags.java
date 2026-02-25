package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;

public final class AprilTags {
    private AprilTags() {}

    private static volatile int detectedCount;
    private static final boolean USE_OPENCV_AXES = false;

    private static final String NUMBER_REGEX =
        "[-+]?(?:\\d+\\.?\\d*|\\d*\\.\\d+)(?:[eE][-+]?\\d+)?";
    private static final Pattern OBJECT_PATTERN = Pattern.compile("\\{[^}]*\\}");
    private static final Pattern ID_PATTERN = Pattern.compile("\"id\"\\s*:\\s*(-?\\d+)");
    private static final Pattern X_PATTERN = Pattern.compile("\"x\"\\s*:\\s*(" + NUMBER_REGEX + ")");
    private static final Pattern Y_PATTERN = Pattern.compile("\"y\"\\s*:\\s*(" + NUMBER_REGEX + ")");
    private static final Pattern Z_PATTERN = Pattern.compile("\"z\"\\s*:\\s*(" + NUMBER_REGEX + ")");
    private static final Pattern ROT_PATTERN = Pattern.compile("\"rot\"\\s*:\\s*\\[([^\\]]*)\\]");

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
    private static final Set<Integer> PROCESSOR_TAG_IDS = Set.of(
        8, 9, 10, 11, 5, 4, 3, 2,
        27, 26, 25, 24, 21, 20, 19, 18
    );

    /**
     * Represents a single AprilTag detection from vision.
     */
    public static final class AprilTagMeasurement {
        public final int id;
        public final Translation3d translation;
        public final Rotation3d rotation;

        /**
         * Create a detection with tag ID, camera-to-tag translation, and rotation.
         *
         * @param id tag ID
         * @param xMeters camera-to-tag X in meters
         * @param yMeters camera-to-tag Y in meters
         * @param zMeters camera-to-tag Z in meters
         * @param rotation camera-to-tag rotation
         */
        public AprilTagMeasurement(int id, double xMeters, double yMeters, double zMeters, Rotation3d rotation) {
            this.id = id;
            this.translation = new Translation3d(xMeters, yMeters, zMeters);
            this.rotation = rotation;
        }
    }

    /**
     * Look up the field pose for a tag ID.
     *
     * @param id tag ID
     * @return field pose if known
     */
    public static Optional<Pose3d> getPose(int id) {
        return Optional.ofNullable(TAG_POSES.get(id));
    }

    /**
     * Get all known field tag poses.
     *
     * @return map of tag ID to field pose
     */
    public static Map<Integer, Pose3d> getAllPoses() {
        return TAG_POSES;
    }

    /**
     * Check if a tag ID is a processor tag.
     *
     * @param id tag ID
     * @return true if the ID is listed as a processor tag
     */
    public static boolean isProcessorTag(int id) {
        return PROCESSOR_TAG_IDS.contains(id);
    }

    /**
     * Store the most recent detected tag count.
     *
     * @param count number of tags detected
     */
    public static void setDetectedCount(int count) {
        detectedCount = Math.max(0, count);
    }

    /**
     * Get the most recent detected tag count.
     *
     * @return detected tag count
     */
    public static int getDetectedCount() {
        return detectedCount;
    }

    /**
     * Parse UDP JSON payload into AprilTag measurements.
     *
     * @param json payload string
     * @return list of measurements (may be empty)
     */
    public static ArrayList<AprilTagMeasurement> parseJson(String json) {
        ArrayList<AprilTagMeasurement> measurements = new ArrayList<>();
        if (json == null || json.isBlank()) {
            return measurements;
        }

        Matcher objMatcher = OBJECT_PATTERN.matcher(json);
        while (objMatcher.find()) {
            String obj = objMatcher.group();
            int id = extractInt(ID_PATTERN, obj, "id");
            double x = extractDouble(X_PATTERN, obj, "x");
            double y = extractDouble(Y_PATTERN, obj, "y");
            double z = extractDouble(Z_PATTERN, obj, "z");
            double[] rot = extractRotation(obj);
            // UDP provides quaternion as w,x,y,z.
            Rotation3d rotation = new Rotation3d(new Quaternion(rot[0], rot[1], rot[2], rot[3]));
            measurements.add(new AprilTagMeasurement(id, x, y, z, rotation));
        }
        return measurements;
    }

    /**
     * Compute robot pose delta vs current pose using two tags (no camera offset).
     *
     * @param tagA first detected tag measurement
     * @param tagB second detected tag measurement
     * @return delta pose (actual minus current), or zero pose if unavailable
     */
    public static Pose2d getRobotOffset(AprilTagMeasurement tagA, AprilTagMeasurement tagB) {
        return getRobotOffset(tagA, tagB, new Translation3d());
    }

    /**
     * Compute robot pose delta vs current pose using a single tag.
     *
     * @param tag detected tag measurement
     * @param cameraToRobotOffset camera-to-robot translation in robot frame
     * @return delta pose (actual minus current), or zero pose if unavailable
     */
    public static Pose2d getRobotOffsetSingleTag(AprilTagMeasurement tag, Translation3d cameraToRobotOffset) {
        Pose2d currentPose = RobotContainer.driveSubsystem.getPose();
        Optional<Pose2d> actualRobotPoseOpt = getRobotPoseSingleTag(tag, cameraToRobotOffset);
        if (actualRobotPoseOpt.isEmpty()) {
            return new Pose2d();
        }
        Pose2d actualRobotPose = actualRobotPoseOpt.get();
        return new Pose2d(
            actualRobotPose.getX() - currentPose.getX(),
            actualRobotPose.getY() - currentPose.getY(),
            actualRobotPose.getRotation().minus(currentPose.getRotation())
        );
    }

    /**
     * Compute robot field pose using a single tag.
     *
     * @param tag detected tag measurement
     * @param cameraToRobotOffset camera-to-robot translation in robot frame
     * @return robot pose if tag ID is known
     */
    public static Optional<Pose2d> getRobotPoseSingleTag(
        AprilTagMeasurement tag,
        Translation3d cameraToRobotOffset
    ) {
        Optional<Pose3d> tagPoseOpt = getPose(tag.id);
        if (tagPoseOpt.isEmpty()) {
            return Optional.empty();
        }

        Pose3d tagPose = tagPoseOpt.get();

        Rotation3d fieldToTagRot = tagPose.getRotation();
        Rotation3d cameraToTagRot = tag.rotation;
        Rotation3d fieldToCameraRot = fieldToTagRot.rotateBy(new Rotation3d(cameraToTagRot.getQuaternion().inverse()));

        Translation3d fieldToTagTrans = tagPose.getTranslation();
        Translation3d cameraToTagTrans = tag.translation;
        Translation3d fieldToCameraTrans = fieldToTagTrans.minus(cameraToTagTrans.rotateBy(fieldToCameraRot));

        Translation3d fieldToRobotTrans = fieldToCameraTrans.plus(cameraToRobotOffset.rotateBy(fieldToCameraRot));
        Rotation2d fieldToRobotYaw = new Rotation2d(fieldToCameraRot.getZ());

        return Optional.of(new Pose2d(fieldToRobotTrans.getX(), fieldToRobotTrans.getY(), fieldToRobotYaw));
    }

    /**
     * Compute robot pose delta vs current pose using two tags.
     *
     * @param tagA first detected tag measurement
     * @param tagB second detected tag measurement
     * @param cameraToRobotOffset camera-to-robot translation in robot frame
     * @return delta pose (actual minus current), or zero pose if unavailable
     */
    public static Pose2d getRobotOffset(
        AprilTagMeasurement tagA,
        AprilTagMeasurement tagB,
        Translation3d cameraToRobotOffset
    ) {
        Pose2d currentPose = RobotContainer.driveSubsystem.getPose();
        Optional<Pose2d> actualRobotPoseOpt = getRobotPose(tagA, tagB, cameraToRobotOffset, currentPose);
        if (actualRobotPoseOpt.isEmpty()) {
            return new Pose2d();
        }
        Pose2d actualRobotPose = actualRobotPoseOpt.get();

        return new Pose2d(
            actualRobotPose.getX() - currentPose.getX(),
            actualRobotPose.getY() - currentPose.getY(),
            actualRobotPose.getRotation().minus(currentPose.getRotation())
        );
    }

    /**
     * Compute robot field pose using two tags.
     *
     * @param tagA first detected tag measurement
     * @param tagB second detected tag measurement
     * @param cameraToRobotOffset camera-to-robot translation in robot frame
     * @return robot pose if both tags are known
     */
    public static Optional<Pose2d> getRobotPose(
        AprilTagMeasurement tagA,
        AprilTagMeasurement tagB,
        Translation3d cameraToRobotOffset
    ) {
        Pose2d currentPose = RobotContainer.driveSubsystem.getPose();
        return getRobotPose(tagA, tagB, cameraToRobotOffset, currentPose);
    }

    /**
     * Compute the field-relative delta to the nearest processor using a single tag.
     *
     * @param tag detected tag measurement
     * @param cameraToRobotOffset camera-to-robot translation in robot frame
     * @return delta translation to nearest processor, if available
     */
    public static Optional<Translation2d> getNearestProcessorDeltaSingleTag(
        AprilTagMeasurement tag,
        Translation3d cameraToRobotOffset
    ) {
        Optional<Pose2d> actualRobotPoseOpt = getRobotPoseSingleTag(tag, cameraToRobotOffset);
        if (actualRobotPoseOpt.isEmpty()) {
            return Optional.empty();
        }
        Optional<Pose3d> processorPoseOpt = getNearestProcessorPose(actualRobotPoseOpt.get());
        if (processorPoseOpt.isEmpty()) {
            return Optional.empty();
        }
        Pose2d robotPose = actualRobotPoseOpt.get();
        Pose3d processorPose = processorPoseOpt.get();
        return Optional.of(new Translation2d(
            processorPose.getX() - robotPose.getX(),
            processorPose.getY() - robotPose.getY()
        ));
    }

    public static Optional<Translation2d> getNearestProcessorDelta(
        AprilTagMeasurement tagA,
        AprilTagMeasurement tagB,
        Translation3d cameraToRobotOffset
    ) {
        Optional<Pose2d> actualRobotPoseOpt = getRobotPose(tagA, tagB, cameraToRobotOffset);
        if (actualRobotPoseOpt.isEmpty()) {
            return Optional.empty();
        }
        Optional<Pose3d> processorPoseOpt = getNearestProcessorPose(actualRobotPoseOpt.get());
        if (processorPoseOpt.isEmpty()) {
            return Optional.empty();
        }
        Pose2d robotPose = actualRobotPoseOpt.get();
        Pose3d processorPose = processorPoseOpt.get();
        return Optional.of(new Translation2d(
            processorPose.getX() - robotPose.getX(),
            processorPose.getY() - robotPose.getY()
        ));
    }

    /**
     * Compute a robot rotation-only offset to face a single tag using the
     * camera-to-tag translation (bearing), not the tag's in-plane rotation.
     *
     * NOTE: This assumes WPILib camera axes (+X forward, +Y left, +Z up) unless
     * USE_OPENCV_AXES is set to true.
     *
     * @param tag detected tag measurement
     * @return pose with rotation set to the bearing-to-tag yaw (translation zero)
     */
    public static Pose2d getRotationOffsetSingleTag(AprilTagMeasurement tag) {
        double x = tag.translation.getX();
        double y = tag.translation.getY();
        double z = tag.translation.getZ();
        Rotation2d yawError;
        if (USE_OPENCV_AXES) {
            if (Math.abs(x) < 1e-6 && Math.abs(z) < 1e-6) {
                return new Pose2d();
            }
            // +X right means a target on the right should be a negative (clockwise) yaw.
            yawError = new Rotation2d(-Math.atan2(x, z));
        } else {
            if (Math.abs(x) < 1e-6 && Math.abs(y) < 1e-6) {
                return new Pose2d();
            }
            yawError = new Rotation2d(Math.atan2(y, x));
        }
        return new Pose2d(0.0, 0.0, yawError);
    }

    /**
     * Compute robot pose from two tags, disambiguating with current pose.
     *
     * @param tagA first detected tag measurement
     * @param tagB second detected tag measurement
     * @param cameraToRobotOffset camera-to-robot translation in robot frame
     * @param currentPose drivetrain pose used to choose triangulation solution
     * @return robot pose if both tags are known
     */
    private static Optional<Pose2d> getRobotPose(
        AprilTagMeasurement tagA,
        AprilTagMeasurement tagB,
        Translation3d cameraToRobotOffset,
        Pose2d currentPose
    ) {
        Optional<Pose3d> tagAPoseOpt = getPose(tagA.id);
        Optional<Pose3d> tagBPoseOpt = getPose(tagB.id);
        if (tagAPoseOpt.isEmpty() || tagBPoseOpt.isEmpty()) {
            return Optional.empty();
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
        return Optional.of(new Pose2d(
            cameraPose.getX() + offsetField.getX(),
            cameraPose.getY() + offsetField.getY(),
            cameraYaw
        ));
    }

    /**
     * Find the closest processor tag to the given robot pose.
     *
     * @param robotPose robot field pose
     * @return nearest processor pose if any are defined
     */
    private static Optional<Pose3d> getNearestProcessorPose(Pose2d robotPose) {
        Pose3d bestPose = null;
        double bestDist = Double.POSITIVE_INFINITY;
        for (int id : PROCESSOR_TAG_IDS) {
            Pose3d pose = TAG_POSES.get(id);
            if (pose == null) {
                continue;
            }
            double dx = pose.getX() - robotPose.getX();
            double dy = pose.getY() - robotPose.getY();
            double dist = Math.hypot(dx, dy);
            if (dist < bestDist) {
                bestDist = dist;
                bestPose = pose;
            }
        }
        return Optional.ofNullable(bestPose);
    }

    /**
     * Helper to build a Pose3d from inches and degrees.
     *
     * @param xIn x in inches
     * @param yIn y in inches
     * @param zIn z in inches
     * @param zRotDeg rotation about Z in degrees
     * @return Pose3d in meters and radians
     */
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

    /**
     * Triangulate camera XY from distances to two tags.
     *
     * @param tagA field pose of tag A
     * @param distA camera-to-tag A distance
     * @param tagB field pose of tag B
     * @param distB camera-to-tag B distance
     * @param currentPose used to select the closest intersection
     * @return camera pose in field coordinates (rotation left as zero)
     */
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

    /**
     * Extract a required integer field from JSON object text.
     *
     * @param pattern regex to match the field
     * @param text object text
     * @param fieldName field name for error messages
     * @return parsed integer
     */
    private static int extractInt(Pattern pattern, String text, String fieldName) {
        Matcher matcher = pattern.matcher(text);
        if (!matcher.find()) {
            throw new IllegalArgumentException("Missing field: " + fieldName);
        }
        return Integer.parseInt(matcher.group(1));
    }

    /**
     * Extract a required double field from JSON object text.
     *
     * @param pattern regex to match the field
     * @param text object text
     * @param fieldName field name for error messages
     * @return parsed double
     */
    private static double extractDouble(Pattern pattern, String text, String fieldName) {
        Matcher matcher = pattern.matcher(text);
        if (!matcher.find()) {
            throw new IllegalArgumentException("Missing field: " + fieldName);
        }
        return Double.parseDouble(matcher.group(1));
    }

    /**
     * Extract a quaternion from a JSON object.
     *
     * @param text object text
     * @return quaternion as {w, x, y, z}
     */
    private static double[] extractRotation(String text) {
        Matcher matcher = ROT_PATTERN.matcher(text);
        if (!matcher.find()) {
            throw new IllegalArgumentException("Missing field: rot");
        }
        String[] parts = matcher.group(1).split(",");
        if (parts.length != 4) {
            throw new IllegalArgumentException("rot must have 4 elements");
        }
        double[] rot = new double[4];
        for (int i = 0; i < 4; i++) {
            rot[i] = Double.parseDouble(parts[i].trim());
        }
        return rot;
    }
}
