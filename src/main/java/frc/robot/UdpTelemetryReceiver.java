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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.AprilTags;

public final class UdpTelemetryReceiver implements AutoCloseable {
    public static volatile Pose2d robotOffset = new Pose2d();
    public static volatile Pose2d robotPose = new Pose2d();
    public static volatile Translation2d processorDelta = new Translation2d();
    public static volatile Translation2d nearestProcessorDelta = new Translation2d();
    public static volatile Pose2d processorTagOffset = new Pose2d();
    public static volatile Rotation2d processorYawError = new Rotation2d();
    public static volatile Rotation2d processorRotateAngle = new Rotation2d();
    public static volatile boolean processorTagDetected;

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

    public static Pose2d getRobotOffset() {
        return robotOffset;
    }

    public static Pose2d getProcessorTagOffset() {
        return processorTagOffset;
    }

    public static Translation2d getNearestProcessorDelta() {
        return nearestProcessorDelta;
    }

    public static Rotation2d getProcessorYawError() {
        return processorYawError;
    }

    public static Rotation2d getProcessorRotateAngle() {
        return processorRotateAngle;
    }

    public static boolean isProcessorTagDetected() {
        return processorTagDetected;
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
                    continue;
                }
                int length = packet.getLength();
                String payload = new String(packet.getData(), packet.getOffset(), length, StandardCharsets.UTF_8);
                lastPacketPublisher.set(payload);
                lastPacketBytesPublisher.set(length);
                lastPacketTimestampPublisher.set(Timer.getFPGATimestamp());
                try {
                    var tags = AprilTags.parseJson(payload);
                    AprilTags.setDetectedCount(tags.size());
                    if (!tags.isEmpty()) {
                        Translation3d cameraToRobotOffset = Constants.cameraToRobotOffset;
                        AprilTags.AprilTagMeasurement tagA = null;
                        AprilTags.AprilTagMeasurement tagB = null;
                        AprilTags.AprilTagMeasurement nearestProcessor = null;
                        double nearestProcessorDist = Double.POSITIVE_INFINITY;
                        for (var tag : tags) {
                            if (AprilTags.getPose(tag.id).isPresent()) {
                                if (tagA == null) {
                                    tagA = tag;
                                } else {
                                    tagB = tag;
                                    break;
                                }
                            }
                        }
                        for (var tag : tags) {
                            if (AprilTags.isProcessorTag(tag.id)) {
                                double dist = Math.hypot(tag.translation.getX(), tag.translation.getY());
                                if (dist < nearestProcessorDist) {
                                    nearestProcessorDist = dist;
                                    nearestProcessor = tag;
                                }
                            }
                        }

                        if (tagA != null && tagB != null) {
                            robotOffset = AprilTags.getRobotOffset(tagA, tagB, cameraToRobotOffset);
                            robotPose = AprilTags.getRobotPose(tagA, tagB, cameraToRobotOffset).orElse(new Pose2d());
                            processorDelta = AprilTags
                                .getNearestProcessorDelta(tagA, tagB, cameraToRobotOffset)
                                .orElse(new Translation2d());
                        } else if (tagA != null) {
                            robotOffset = AprilTags.getRobotOffsetSingleTag(tagA, cameraToRobotOffset);
                            robotPose = AprilTags.getRobotPoseSingleTag(tagA, cameraToRobotOffset).orElse(new Pose2d());
                            processorDelta = AprilTags
                                .getNearestProcessorDeltaSingleTag(tagA, cameraToRobotOffset)
                                .orElse(new Translation2d());
                        } else {
                            robotOffset = new Pose2d();
                            robotPose = new Pose2d();
                            processorDelta = new Translation2d();
                        }

                        if (nearestProcessor != null) {
                            processorTagDetected = true;
                            processorTagOffset = computeProcessorTagOffset(nearestProcessor, null, cameraToRobotOffset);
                            nearestProcessorDelta = computeNearestProcessorDelta(nearestProcessor, cameraToRobotOffset);
                            processorRotateAngle = computeProcessorRotateAngle(nearestProcessorDelta);
                            processorYawError = processorRotateAngle;
                        } else {
                            processorTagDetected = false;
                            processorTagOffset = new Pose2d();
                            processorYawError = new Rotation2d();
                            processorRotateAngle = new Rotation2d();
                            nearestProcessorDelta = new Translation2d();
                        }
                    } else {
                        robotOffset = new Pose2d();
                        robotPose = new Pose2d();
                        processorDelta = new Translation2d();
                        processorTagDetected = false;
                        processorTagOffset = new Pose2d();
                        processorYawError = new Rotation2d();
                        processorRotateAngle = new Rotation2d();
                        nearestProcessorDelta = new Translation2d();
                    }
                } catch (RuntimeException ex) {
                    AprilTags.setDetectedCount(0);
                    robotOffset = new Pose2d();
                    robotPose = new Pose2d();
                    processorDelta = new Translation2d();
                    processorTagDetected = false;
                    processorTagOffset = new Pose2d();
                    processorYawError = new Rotation2d();
                    processorRotateAngle = new Rotation2d();
                    nearestProcessorDelta = new Translation2d();
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

    private static Pose2d computeProcessorTagOffset(
        AprilTags.AprilTagMeasurement tagA,
        AprilTags.AprilTagMeasurement tagB,
        Translation3d cameraToRobotOffset
    ) {
        var robotPoseOpt = tagB == null
            ? AprilTags.getRobotPoseSingleTag(tagA, cameraToRobotOffset)
            : AprilTags.getRobotPose(tagA, tagB, cameraToRobotOffset);
        if (robotPoseOpt.isEmpty()) {
            return new Pose2d();
        }
        Pose2d robotPose = robotPoseOpt.get();

        AprilTags.AprilTagMeasurement targetTag = tagA;
        if (tagB != null) {
            var poseA = AprilTags.getPose(tagA.id);
            var poseB = AprilTags.getPose(tagB.id);
            if (poseA.isPresent() && poseB.isPresent()) {
                Translation2d a = new Translation2d(poseA.get().getX(), poseA.get().getY());
                Translation2d b = new Translation2d(poseB.get().getX(), poseB.get().getY());
                double distA = robotPose.getTranslation().getDistance(a);
                double distB = robotPose.getTranslation().getDistance(b);
                targetTag = distB < distA ? tagB : tagA;
            } else if (poseA.isEmpty() && poseB.isPresent()) {
                targetTag = tagB;
            }
        }

        var targetPoseOpt = AprilTags.getPose(targetTag.id);
        if (targetPoseOpt.isEmpty()) {
            return new Pose2d();
        }
        Pose3d targetPose = targetPoseOpt.get();
        return new Pose2d(
            targetPose.getX() - robotPose.getX(),
            targetPose.getY() - robotPose.getY(),
            new Rotation2d()
        );
    }

    private static Translation2d computeNearestProcessorDelta(
        AprilTags.AprilTagMeasurement tag,
        Translation3d cameraToRobotOffset
    ) {
        Translation3d robotToTag = tag.translation.minus(cameraToRobotOffset);
        return new Translation2d(robotToTag.getX(), robotToTag.getY());
    }

    private static Rotation2d computeProcessorRotateAngle(Translation2d delta) {
        return new Rotation2d(-Math.atan2(delta.getY(), -delta.getX()));
    }

    private static Rotation2d computeProcessorYawError(
        AprilTags.AprilTagMeasurement tagA,
        AprilTags.AprilTagMeasurement tagB,
        Translation3d cameraToRobotOffset
    ) {
        AprilTags.AprilTagMeasurement targetTag = tagA;
        if (tagB != null) {
            double distA = Math.hypot(tagA.translation.getX(), tagA.translation.getY());
            double distB = Math.hypot(tagB.translation.getX(), tagB.translation.getY());
            targetTag = distB < distA ? tagB : tagA;
        }
        Translation2d delta = computeNearestProcessorDelta(targetTag, cameraToRobotOffset);
        return computeProcessorRotateAngle(delta);
    }
}
