package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.UdpTelemetryReceiver;

public class Shooter extends SubsystemBase {
    private final CommandXboxController joystick;

    private static final double kP = 0.0002;
    private static final double kI = 0.001;
    private static final double kD = 0.000001;
    private static final double MAX_OUTPUT = 1.0;
    private static final double MIN_OUTPUT = -1.0;

    private static final double MIN_TARGET_RPM = 2000.0;
    private static final double MAX_TARGET_RPM = 5000.0;
    private static final double CONSTANT_TARGET_RPM = 3500.0;
    private static final double FEEDER_POWER = 0.2;

    private static final InterpolatingDoubleTreeMap RPM_CURVE = new InterpolatingDoubleTreeMap();
    static {
        RPM_CURVE.put(1.0, 2000.0);
        RPM_CURVE.put(2.0, 3000.0);
        RPM_CURVE.put(3.0, 4000.0);
        RPM_CURVE.put(4.0, 5000.0);
    }

    private final PIDController pid = new PIDController(kP, kI, kD);

    public static volatile double currentRpm = 0.0;
    public static volatile double targetRpm = 0.0;
    public static volatile double feederPower = 0.0;
    public static volatile double shooterOutput = 0.0;
    
    public Shooter(CommandXboxController joystick)
    {
        this.joystick=joystick;
    }

    // Temporary ids, but i think 23-26 work since Vansh's intake is 27-29
    public static final SparkMax shooter1 = new SparkMax(23, MotorType.kBrushless);
    public static final SparkMax shooter2 = new SparkMax(24, MotorType.kBrushless);
    public static final SparkMax feeder1 = new SparkMax(25, MotorType.kBrushless);
    public static final SparkMax feeder2 = new SparkMax(26, MotorType.kBrushless);
    
    public static final RelativeEncoder encoder1 = shooter1.getEncoder();
    public static final RelativeEncoder encoder2 = shooter2.getEncoder();

    public static void setShooterPower(double power) {
        shooter1.set(power);
        shooter2.set(power);
        shooterOutput = power;
    }

    public static void setFeederPower(double power) {
        feeder1.set(power);
        feeder2.set(power);
        feederPower = power;
    }

    private double getAverageRpm() {
        return (encoder1.getVelocity() + encoder2.getVelocity()) / 2.0;
    }

    private double rpmFromDistance(double distanceMeters) {
        double rpm = RPM_CURVE.get(distanceMeters);
        return MathUtil.clamp(rpm, MIN_TARGET_RPM, MAX_TARGET_RPM);
    }

    public Command getDefaultCommand() {
       return Commands.run(() -> {
            double newTargetRpm = 0.0;
            if (joystick.x().getAsBoolean()) {
                if (UdpTelemetryReceiver.isProcessorTagDetected()) {
                    double distanceMeters = UdpTelemetryReceiver.getProcessorDistanceMeters();
                    if (Double.isFinite(distanceMeters)) {
                        newTargetRpm = rpmFromDistance(distanceMeters);
                    }
                }
            } else if (joystick.y().getAsBoolean()) {
                newTargetRpm = CONSTANT_TARGET_RPM;
            }

            targetRpm = newTargetRpm;

            if (targetRpm == 0.0) {
                pid.reset();
                setShooterPower(0.0);
                setFeederPower(0.0);
                currentRpm = getAverageRpm();
                return;
            }

            currentRpm = getAverageRpm();
            double output = pid.calculate(currentRpm, targetRpm);
            output = Math.max(MIN_OUTPUT, Math.min(MAX_OUTPUT, output));
            setShooterPower(0.5*output); // scale
            setFeederPower(FEEDER_POWER);
       }, this);
    }

    @Override
    public void periodic() {
        currentRpm = getAverageRpm();
    }
}
