package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.UdpTelemetryReceiver;
import frc.robot.constants.Constants;

public class Shooter extends SubsystemBase {
    private final CommandXboxController joystick;

    private static final double MIN_TARGET_RPM = 2000.0;
    private static final double MAX_TARGET_RPM = 5000.0;
    private static final double CONSTANT_TARGET_RPM = 3500.0;
    private static final double FEEDER_POWER = 0.2;

    private static final InterpolatingDoubleTreeMap BLUE_RPM_CURVE = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap GREEN_RPM_CURVE = new InterpolatingDoubleTreeMap();
    static {
        BLUE_RPM_CURVE.put(1.0, 2000.0);
        BLUE_RPM_CURVE.put(2.0, 3000.0);
        BLUE_RPM_CURVE.put(3.0, 4000.0);
        BLUE_RPM_CURVE.put(4.0, 5000.0);

        GREEN_RPM_CURVE.put(1.0, 2000.0);
        GREEN_RPM_CURVE.put(2.0, 3000.0);
        GREEN_RPM_CURVE.put(3.0, 4000.0);
        GREEN_RPM_CURVE.put(4.0, 5000.0);
    }

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    public static volatile double currentBlueRpm = 0.0;
    public static volatile double currentGreenRpm = 0.0;
    public static volatile double targetBlueRpm = 0.0;
    public static volatile double targetGreenRpm = 0.0;
    public static volatile double feederPower = 0.0;
    public static volatile double shooterOutput = 0.0;
    
    public Shooter(CommandXboxController joystick)
    {
        this.joystick=joystick;

        TalonFXConfiguration blueConfig = new TalonFXConfiguration();
        blueConfig.Slot0.kP = Constants.shooterKp;
        blueConfig.Slot0.kI = Constants.shooterKi;
        blueConfig.Slot0.kD = Constants.shooterKd;
        blueConfig.Slot0.kV = Constants.shooterKv;
        bluemotor.getConfigurator().apply(blueConfig);

        TalonFXConfiguration greenConfig = new TalonFXConfiguration();
        greenConfig.Slot0.kP = Constants.shooterKp;
        greenConfig.Slot0.kI = Constants.shooterKi;
        greenConfig.Slot0.kD = Constants.shooterKd;
        greenConfig.Slot0.kV = Constants.shooterKv;
        greenConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        greenmotor.getConfigurator().apply(greenConfig);
    }

    // Temporary ids, but i think 23-26 work since Vansh's intake is 27-29
    public static final TalonFX bluemotor = new TalonFX(21);
    public static final TalonFX greenmotor = new TalonFX(22);
    public static final SparkMax feeder1 = new SparkMax(23, MotorType.kBrushless);
    public static final SparkMax feeder2 = new SparkMax(24, MotorType.kBrushless);


    private void setShooterRps(double blueRps, double greenRps) {
        bluemotor.setControl(velocityRequest.withVelocity(blueRps));
        greenmotor.setControl(velocityRequest.withVelocity(greenRps));
        targetBlueRpm = blueRps * 60.0;
        targetGreenRpm = greenRps * 60.0;
    }

    public static void setFeederPower(double power) {
        feeder1.set(power);
        feeder2.set(power);
        feederPower = power;
    }

    private double blueRpmFromDistance(double distanceMeters) {
        double rpm = BLUE_RPM_CURVE.get(distanceMeters);
        return MathUtil.clamp(rpm, MIN_TARGET_RPM, MAX_TARGET_RPM);
    }

    private double greenRpmFromDistance(double distanceMeters) {
        double rpm = GREEN_RPM_CURVE.get(distanceMeters);
        return MathUtil.clamp(rpm, MIN_TARGET_RPM, MAX_TARGET_RPM);
    }

    public Command getDefaultCommand() {
       return Commands.run(() -> {
            double newTargetBlueRpm = 0.0;
            double newTargetGreenRpm = 0.0;
            if (joystick.x().getAsBoolean()) {
                if (UdpTelemetryReceiver.isProcessorTagDetected()) {
                    double distanceMeters = UdpTelemetryReceiver.getProcessorDistanceMeters();
                    if (Double.isFinite(distanceMeters)) {
                        newTargetBlueRpm = blueRpmFromDistance(distanceMeters);
                        newTargetGreenRpm = greenRpmFromDistance(distanceMeters);
                    }
                }
            } else if (joystick.y().getAsBoolean()) {
                newTargetBlueRpm = CONSTANT_TARGET_RPM;
                newTargetGreenRpm = CONSTANT_TARGET_RPM;
            }

            if (newTargetBlueRpm == 0.0 && newTargetGreenRpm == 0.0) {
                setShooterRps(0.0, 0.0);
                setFeederPower(0.0);
                return;
            }
            setShooterRps(newTargetBlueRpm / 60.0, newTargetGreenRpm / 60.0);
            setFeederPower(FEEDER_POWER);
       }, this);
    }

    @Override
    public void periodic() {
        currentBlueRpm = bluemotor.getVelocity().getValueAsDouble() * 60.0;
        currentGreenRpm = greenmotor.getVelocity().getValueAsDouble() * 60.0;
        shooterOutput = bluemotor.getDutyCycle().getValueAsDouble();
    }
}
