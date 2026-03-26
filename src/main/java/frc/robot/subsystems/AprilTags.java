package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * APRILTAG/CAMERA DETECTION DISABLED FOR COMP 2.
 * Kept as a stub to avoid breaking older references if re-enabled later.
 */
public final class AprilTags {
    private AprilTags() {}

    private static volatile int detectedCount;

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

    public static boolean isProcessorTag(int id) {
        return false;
    }

    public static void setDetectedCount(int count) {
        detectedCount = Math.max(0, count);
    }

    public static int getDetectedCount() {
        return detectedCount;
    }

    public static ArrayList<AprilTagMeasurement> parseJson(String json) {
        return new ArrayList<>();
    }
}
