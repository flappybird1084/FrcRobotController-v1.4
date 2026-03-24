package frc.robot.constants;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import frc.robot.constants.TunerConstants;

public class Constants {
    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double scaling = 0.4;

    public static final int pigeonID = 15;
    public static final Pigeon2 imu = new Pigeon2(Constants.pigeonID);

    public static final int udpTelemetryPort = 5800;

    // Camera-to-robot transform in robot frame (meters) and camera yaw offset in field frame.
    public static final Translation3d cameraToRobotOffset = new Translation3d(0.0, 0.0, 0.0);
    // as for this camera offset, i would ask AI what xyz mean in this context. i'm not entirely sure that x and y are field translation, y might be height...
    public static final Rotation2d cameraFieldRotation = new Rotation2d(0.0);
    public static final List<Double> processorRotateAngleLimitDeg = List.of(-30.0, 30.0);

    // Multiplier applied to the raw camera-to-tag distance before feeding into the shooter RPM curve.
    // Tune this to compensate for camera mounting offset or field measurement discrepancies.
    public static final double shooterDistanceScale = 1.31;

    // Flat offset added to the scaled distance (meters).
    // Accounts for the physical gap between the camera and the shooter exit point.
    public static final double shooterDistanceBias = 0.5;

    public static final int blueShooterMotorID = 21;
    public static final int greenShooterMotorID = 22;

    public static final int shooterFeederMotor1ID = 23;
    public static final int shooterFeederMotor2ID = 24;
   
    public static final int intakeMotorID = 26;
    public static final int pivotMotorID = 25;

    // Shooter flywheel tuning
    public static final double shooterMaxRpm = 5000.0;
    public static final double shooterMaxRps = shooterMaxRpm / 60.0;
    public static final double shooterJoystickDeadband = 0.05;

    // Slot 0 PID gains for TalonFX onboard velocity control (1 kHz loop).
    // kV = 12V / shooterMaxRps — voltage per RPS at steady state. Tune with SysId.
    // Start kI and kD at zero; only add if steady-state error or oscillation persists.
    public static final double shooterKp = 0.11;
    public static final double shooterKi = 0.0;
    public static final double shooterKd = 0.0;
    public static final double shooterKv = 12.0 / shooterMaxRps;
}
