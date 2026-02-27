package frc.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.AprilTags;

public final class UdpTelemetryReceiver implements AutoCloseable {
    public static volatile Rotation2d processorYawError = new Rotation2d();
    public static volatile Rotation2d processorRotateAngle = new Rotation2d();
    public static volatile boolean processorTagDetected;
    public static volatile boolean processorYawValid;
    public static volatile double secondsSinceLastTag = Double.POSITIVE_INFINITY;
    private static volatile double lastTagTimestamp = -1.0;

    private final int port;
    private final NetworkTable table;
    private final StringPublisher lastPacketPublisher;
    private final DoublePublisher lastPacketBytesPublisher;
    private final DoublePublisher lastPacketTimestampPublisher;

    private volatile boolean running;
    private Thread thread;
    private DatagramSocket socket;

    public UdpTelemetryReceiver(int port) {
        this.port = port;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        this.table = inst.getTable("Udp");
        this.lastPacketPublisher = table.getStringTopic("LastPacket").publish();
        this.lastPacketBytesPublisher = table.getDoubleTopic("LastPacketBytes").publish();
        this.lastPacketTimestampPublisher = table.getDoubleTopic("LastPacketTimestamp").publish();
    }

    public static Rotation2d getProcessorYawError() {
        return processorYawError;
    }

    public static Rotation2d getProcessorRotateAngle() {
        return processorRotateAngle;
    }

    public static boolean isProcessorYawValid() {
        return processorYawValid;
    }

    public static boolean isProcessorTagDetected() {
        return processorTagDetected;
    }

    public static double getSecondsSinceLastTag() {
        return secondsSinceLastTag;
    }

    public void start() {
        if (thread != null) {
            return;
        }
        running = true;
        thread = new Thread(this::runLoop, "UdpTelemetryReceiver");
        thread.setDaemon(true);
        thread.start();
    }

    private void runLoop() {
        try (DatagramSocket datagramSocket = new DatagramSocket(port)) {
            socket = datagramSocket;
            socket.setSoTimeout(200);
            byte[] buffer = new byte[2048];
            while (running) {
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                try {
                    socket.receive(packet);
                } catch (SocketTimeoutException timeout) {
                    updateSecondsSinceLastTag(Timer.getFPGATimestamp());
                    continue;
                }
                int length = packet.getLength();
                String payload = new String(packet.getData(), packet.getOffset(), length, StandardCharsets.UTF_8);
                double now = Timer.getFPGATimestamp();
                lastPacketPublisher.set(payload);
                lastPacketBytesPublisher.set(length);
                lastPacketTimestampPublisher.set(now);
                try {
                    var tags = AprilTags.parseJson(payload);
                    AprilTags.setDetectedCount(tags.size());
                    if (!tags.isEmpty()) {
                        lastTagTimestamp = now;
                        AprilTags.AprilTagMeasurement nearestProcessor = null;
                        double nearestProcessorDist = Double.POSITIVE_INFINITY;
                        for (var tag : tags) {
                            if (AprilTags.isProcessorTag(tag.id)) {
                                double dist = Math.hypot(tag.translation.getX(), tag.translation.getZ());
                                if (dist < nearestProcessorDist) {
                                    nearestProcessorDist = dist;
                                    nearestProcessor = tag;
                                }
                            }
                        }

                        if (nearestProcessor != null) {
                            processorTagDetected = true;
                            Rotation2d rotateAngle = computeProcessorRotateAngle(nearestProcessor);
                            processorRotateAngle = rotateAngle;
                            processorYawValid = isProcessorRotateAngleWithinLimit(rotateAngle);
                            processorYawError = processorYawValid ? processorRotateAngle : new Rotation2d();
                        } else {
                            processorTagDetected = false;
                            processorYawError = new Rotation2d();
                            processorRotateAngle = new Rotation2d();
                            processorYawValid = false;
                        }
                    } else {
                        processorTagDetected = false;
                        processorYawError = new Rotation2d();
                        processorRotateAngle = new Rotation2d();
                        processorYawValid = false;
                    }
                } catch (RuntimeException ex) {
                    AprilTags.setDetectedCount(0);
                    processorTagDetected = false;
                    processorYawError = new Rotation2d();
                    processorRotateAngle = new Rotation2d();
                    processorYawValid = false;
                } finally {
                    updateSecondsSinceLastTag(now);
                }
            }
        } catch (IOException ex) {
            lastPacketPublisher.set("UDP error: " + ex.getMessage());
        } finally {
            socket = null;
        }
    }

    @Override
    public void close() {
        running = false;
        if (socket != null) {
            socket.close();
        }
    }

    private static Rotation2d computeProcessorRotateAngle(AprilTags.AprilTagMeasurement tag) {
        return new Rotation2d(
            -Math.atan2(tag.translation.getX(), tag.translation.getZ())
                + tag.rotation.getZ()
        );
    }

    private static void updateSecondsSinceLastTag(double now) {
        if (lastTagTimestamp < 0.0) {
            secondsSinceLastTag = Double.POSITIVE_INFINITY;
        } else {
            secondsSinceLastTag = Math.max(0.0, now - lastTagTimestamp);
        }
    }

    private static boolean isProcessorRotateAngleWithinLimit(Rotation2d rotateAngle) {
        double angleDeg = rotateAngle.getDegrees();
        double minDeg = Constants.processorRotateAngleLimitDeg.get(0);
        double maxDeg = Constants.processorRotateAngleLimitDeg.get(1);
        return angleDeg >= minDeg && angleDeg <= maxDeg;
    }
}
