package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * APRILTAG/CAMERA TELEMETRY DISABLED FOR COMP 2.
 * Kept as a stub to avoid breaking older references if re-enabled later.
 */
public final class UdpTelemetryReceiver implements AutoCloseable {
    public UdpTelemetryReceiver(int port) {}

    public static Rotation2d getProcessorYawError() {
        return new Rotation2d();
    }

    public static Rotation2d getProcessorRotateAngle() {
        return new Rotation2d();
    }

    public static boolean isProcessorYawValid() {
        return false;
    }

    public static boolean isProcessorTagDetected() {
        return false;
    }

    public static double getSecondsSinceLastTag() {
        return Double.POSITIVE_INFINITY;
    }

    public static double getNearestProcessorDistMeters() {
        return Double.POSITIVE_INFINITY;
    }

    public void start() {}

    @Override
    public void close() {}
}
