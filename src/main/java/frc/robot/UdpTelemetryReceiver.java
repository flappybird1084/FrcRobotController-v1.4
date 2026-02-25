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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.AprilTags;

public final class UdpTelemetryReceiver implements AutoCloseable {
    public static volatile Pose2d robotOffset = new Pose2d();
    public static volatile Pose2d robotPose = new Pose2d();
    public static volatile Translation2d processorDelta = new Translation2d();
    public static volatile Pose2d processorTagOffset = new Pose2d();
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
                        Translation3d cameraToRobotOffset = new Translation3d();
                        AprilTags.AprilTagMeasurement tagA = null;
                        AprilTags.AprilTagMeasurement tagB = null;
                        AprilTags.AprilTagMeasurement procA = null;
                        AprilTags.AprilTagMeasurement procB = null;
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
                                if (procA == null) {
                                    procA = tag;
                                } else {
                                    procB = tag;
                                    break;
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

                        if (procA != null) {
                            processorTagDetected = true;
                            AprilTags.AprilTagMeasurement bestTag = procA;
                            if (procB != null && procB.translation.getNorm() < procA.translation.getNorm()) {
                                bestTag = procB;
                            }
                            processorTagOffset = AprilTags.getRotationOffsetSingleTag(bestTag);
                        } else {
                            processorTagDetected = false;
                            processorTagOffset = new Pose2d();
                        }
                    } else {
                        robotOffset = new Pose2d();
                        robotPose = new Pose2d();
                        processorDelta = new Translation2d();
                        processorTagDetected = false;
                        processorTagOffset = new Pose2d();
                    }
                } catch (RuntimeException ex) {
                    AprilTags.setDetectedCount(0);
                    robotOffset = new Pose2d();
                    robotPose = new Pose2d();
                    processorDelta = new Translation2d();
                    processorTagDetected = false;
                    processorTagOffset = new Pose2d();
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
}
