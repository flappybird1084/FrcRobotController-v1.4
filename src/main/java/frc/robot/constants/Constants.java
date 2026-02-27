package frc.robot.constants;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import frc.robot.constants.TunerConstants;

public class Constants {
    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double scaling = 0.3;

    public static final int pigeonID = 15;
    public static final Pigeon2 imu = new Pigeon2(Constants.pigeonID);

    public static final int udpTelemetryPort = 5800;

    // Camera-to-robot transform in robot frame (meters) and camera yaw offset in field frame.
    public static final Translation3d cameraToRobotOffset = new Translation3d(0.0, 0.0, 0.0);
    public static final Rotation2d cameraFieldRotation = new Rotation2d(0.0);
    public static final List<Double> processorRotateAngleLimitDeg = List.of(-30.0, 30.0);
}
