package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.UdpTelemetryReceiver;
import frc.robot.constants.Constants;

public class Shooter extends SubsystemBase {
    private final CommandXboxController joystick;


    // Distance (meters, after scale+bias applied) → target RPM.
    // Replace placeholder points with real measured values; interpolation fills in between.
    private static final InterpolatingDoubleTreeMap RPM_CURVE = new InterpolatingDoubleTreeMap();
    static {
        RPM_CURVE.put(1.0, 1500.0);
        RPM_CURVE.put(2.0, 2500.0);
        RPM_CURVE.put(3.0, 3500.0);
        RPM_CURVE.put(4.0, 4500.0);
    }

    public static double targetRpm   = 0.0;
    public static double currentRpm  = 0.0;
    public static double shooterPower = 0.0;

    public static final TalonFX blueMotor  = new TalonFX(Constants.blueShooterMotorID);
    public static final TalonFX greenMotor = new TalonFX(Constants.greenShooterMotorID);

    // Feeders use simple power control — no velocity loop needed.
    public static final SparkMax feeder1 = new SparkMax(Constants.shooterFeederMotor1ID, MotorType.kBrushless);
    public static final SparkMax feeder2 = new SparkMax(Constants.shooterFeederMotor2ID, MotorType.kBrushless);

    // Reusable control request — mutated each loop via withVelocity().
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    public Shooter(CommandXboxController joystick) {
        this.joystick = joystick;

        TalonFXConfiguration blueConfig = new TalonFXConfiguration();
        blueConfig.Slot0.kP = Constants.shooterKp;
        blueConfig.Slot0.kI = Constants.shooterKi;
        blueConfig.Slot0.kD = Constants.shooterKd;
        blueConfig.Slot0.kV = Constants.shooterKv;
        blueMotor.getConfigurator().apply(blueConfig);

        // Green spins opposite to blue so both wheels propel the game piece the same way.
        // Same PID gains, but Clockwise_Positive so a positive RPS target spins it the other direction.
        TalonFXConfiguration greenConfig = new TalonFXConfiguration();
        greenConfig.Slot0.kP = Constants.shooterKp;
        greenConfig.Slot0.kI = Constants.shooterKi;
        greenConfig.Slot0.kD = Constants.shooterKd;
        greenConfig.Slot0.kV = Constants.shooterKv;
        greenConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        greenMotor.getConfigurator().apply(greenConfig);
    }

    /** Send both flywheel motors to the same velocity. Updates targetRpm for telemetry. */
    private void setShooterRps(double rps) {
        blueMotor.setControl(velocityRequest.withVelocity(rps));
        greenMotor.setControl(velocityRequest.withVelocity(rps));
        targetRpm = rps * 60.0;
    }

    public static void setFeederPower(double power) {
        feeder1.set(power);
        feeder2.set(power);
    }

    /** Manual joystick control — active whenever no other command requires this subsystem. */
    public Command getDefaultCommand() {
        return new RunCommand(() -> {
            double input = joystick.getRightY();
            if (Math.abs(input) < Constants.shooterJoystickDeadband) input = 0.0;
            setShooterRps(input * Constants.shooterMaxRps);
        }, this);
    }

    /**
     * Sets RPM from the AprilTag distance curve.
     * Designed to be bound to a button with whileTrue() — default command resumes on release.
     */
    public Command autoRpmFromDistanceCommand() {
        return new RunCommand(() -> {
            double dist = UdpTelemetryReceiver.nearestProcessorDistMeters
                        * Constants.shooterDistanceScale + Constants.shooterDistanceBias;
            double rpm = Double.isFinite(dist) ? RPM_CURVE.get(dist) : 0.0;
            setShooterRps(rpm / 60.0);
        }, this);
    }

    /** Always runs — only reads sensors and updates telemetry fields, never commands motors. */
    @Override
    public void periodic() {
        currentRpm   = blueMotor.getVelocity().getValueAsDouble() * 60.0;
        shooterPower = blueMotor.getDutyCycle().getValueAsDouble();
    }
}
