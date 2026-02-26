package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    // ids are 3 and 19 for the shooter (green is id 19, blue is id 3)
    private static final int GREEN_ID = 19;
    private static final int BLUE_ID = 3;

    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 6000.0;
    private static final double MIN_RPM_STEP = 50.0;
    private static final double MAX_RPM_STEP = 500.0;

    private final TalonFX greenMotor = new TalonFX(GREEN_ID);
    private final TalonFX blueMotor = new TalonFX(BLUE_ID);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0.0);

    private double targetRpm = 0.0;
    private double rpmStep = 250.0;
    private boolean enabled = false;

    public Shooter() {
        TalonFXConfiguration greenConfig = new TalonFXConfiguration();
        greenConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        greenMotor.getConfigurator().apply(greenConfig);

        TalonFXConfiguration blueConfig = new TalonFXConfiguration();
        blueConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        blueMotor.getConfigurator().apply(blueConfig);
    }

    public Command getDefaultCommand() {
        return run(this::runShooter);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            setTargetRpm(0.0);
        }
    }

    public void increaseRpm() {
        setTargetRpm(targetRpm + rpmStep);
    }

    public void decreaseRpm() {
        setTargetRpm(targetRpm - rpmStep);
    }

    public void setTargetRpm(double rpm) {
        targetRpm = clamp(rpm, MIN_RPM, MAX_RPM);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void increaseRpmStep() {
        setRpmStep(rpmStep + 50.0);
    }

    public void decreaseRpmStep() {
        setRpmStep(rpmStep - 50.0);
    }

    public void setRpmStep(double step) {
        rpmStep = clamp(step, MIN_RPM_STEP, MAX_RPM_STEP);
    }

    public double getRpmStep() {
        return rpmStep;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public double getGreenRpm() {
        return greenMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    public double getBlueRpm() {
        return blueMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    public double getAverageRpm() {
        return (getGreenRpm() + getBlueRpm()) / 2.0;
    }

    public double getAppliedVoltage() {
        return greenMotor.getMotorVoltage().getValueAsDouble();
    }

    private void runShooter() {
        if (!enabled) {
            greenMotor.set(0.0);
            blueMotor.set(0.0);
            return;
        }

        double targetRps = targetRpm / 60.0;
        greenMotor.setControl(velocityControl.withVelocity(targetRps));
        blueMotor.setControl(velocityControl.withVelocity(targetRps));
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
