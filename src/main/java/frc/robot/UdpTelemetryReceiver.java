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

public final class UdpTelemetryReceiver implements AutoCloseable {
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
