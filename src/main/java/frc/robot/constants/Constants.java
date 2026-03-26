package frc.robot.constants;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.constants.TunerConstants;

public class Constants {
    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double scaling = 0.4;

    public static final int pigeonID = 15;
    public static final Pigeon2 imu = new Pigeon2(Constants.pigeonID);

    // public static final int udpTelemetryPort = 5800; // APRILTAG/CAMERA DISABLED FOR COMP 2

    // public static final Translation3d cameraToRobotOffset = new Translation3d(0.0, 0.0, 0.0); // APRILTAG/CAMERA DISABLED FOR COMP 2
    // public static final Rotation2d cameraFieldRotation = new Rotation2d(0.0); // APRILTAG/CAMERA DISABLED FOR COMP 2
    // public static final List<Double> processorRotateAngleLimitDeg = List.of(-30.0, 30.0); // APRILTAG/CAMERA DISABLED FOR COMP 2

    // public static final double shooterDistanceScale = 1.31; // APRILTAG/CAMERA DISABLED FOR COMP 2

    // public static final double shooterDistanceBias = 0.5; // APRILTAG/CAMERA DISABLED FOR COMP 2

    public static final int blueShooterMotorID = 21;
    public static final int greenShooterMotorID = 22;
    public static final int shooterFeederMotorID = 23; // CHANGE THIS AT PRACTICE if feeder CAN ID differs.

    // public static final int shooterFeederMotor1ID = 23; // OLD DUAL-FEEDER CONFIG
    // public static final int shooterFeederMotor2ID = 24; // OLD DUAL-FEEDER CONFIG
   
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

    // Open-loop shooter/feeder scaling for comp controls.
    public static final double shooterStickScale = 0.85; // CHANGE THIS AT PRACTICE.
    public static final double feederStickScale = 0.85; // CHANGE THIS AT PRACTICE.
    public static final double blueShooterDirection = 1.0; // CHANGE THIS AT PRACTICE if spin direction is backwards.
    public static final double greenShooterDirection = -1.0; // CHANGE THIS AT PRACTICE if spin direction is backwards.
    public static final double feederDirection = 1.0; // CHANGE THIS AT PRACTICE if feed direction is backwards.

    // Comp "back off from wall" tuning values.
    public static final double compBackoffMeters = 1.5; // CHANGE THIS AT PRACTICE.
    public static final double compBackoffMaxVel = 0.5; // CHANGE THIS AT PRACTICE.
    public static final double compBackoffMaxAccel = 0.5; // CHANGE THIS AT PRACTICE.
}
