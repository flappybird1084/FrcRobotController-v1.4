package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class AprilTags {
    private AprilTags() {}

    private static volatile int detectedCount;

    private static final String NUMBER_REGEX =
        "[-+]?(?:\\d+\\.?\\d*|\\d*\\.\\d+)(?:[eE][-+]?\\d+)?";
    private static final Pattern OBJECT_PATTERN = Pattern.compile("\\{[^}]*\\}");
    private static final Pattern ID_PATTERN = Pattern.compile("\"id\"\\s*:\\s*(-?\\d+)");
    private static final Pattern X_PATTERN = Pattern.compile("\"x\"\\s*:\\s*(" + NUMBER_REGEX + ")");
    private static final Pattern Y_PATTERN = Pattern.compile("\"y\"\\s*:\\s*(" + NUMBER_REGEX + ")");
    private static final Pattern Z_PATTERN = Pattern.compile("\"z\"\\s*:\\s*(" + NUMBER_REGEX + ")");
    private static final Pattern ROT_PATTERN = Pattern.compile("\"rot\"\\s*:\\s*\\[([^\\]]*)\\]");

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
