package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    // ids are 3 and 19 for the shooter (green is id 19, blue is id 3)
    private static final int GREEN_ID = 19;
    private static final int BLUE_ID = 3;

    private static final double MIN_VOLTAGE = 0.0;
    private static final double MAX_VOLTAGE = 12.0;
    private static final double MIN_STEP = 0.05;
    private static final double MAX_STEP = 2.0;

    private final TalonFX greenMotor = new TalonFX(GREEN_ID);
    private final TalonFX blueMotor = new TalonFX(BLUE_ID);
    private final VoltageOut voltageControl = new VoltageOut(0.0);

    private double targetVoltage = 0.0;
    private double voltageStep = 0.25;
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
            setVoltage(0.0);
        }
    }

    public void increaseVoltage() {
        setVoltage(targetVoltage + voltageStep);
    }

    public void decreaseVoltage() {
        setVoltage(targetVoltage - voltageStep);
    }

    public void setVoltage(double voltage) {
        targetVoltage = clamp(voltage, MIN_VOLTAGE, MAX_VOLTAGE);
    }

    public double getTargetVoltage() {
        return targetVoltage;
    }

    public void increaseVoltageStep() {
        setVoltageStep(voltageStep + 0.05);
    }

    public void decreaseVoltageStep() {
        setVoltageStep(voltageStep - 0.05);
    }

    public void setVoltageStep(double step) {
        voltageStep = clamp(step, MIN_STEP, MAX_STEP);
    }

    public double getVoltageStep() {
        return voltageStep;
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

        greenMotor.setControl(voltageControl.withOutput(targetVoltage));
        blueMotor.setControl(voltageControl.withOutput(targetVoltage));
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
