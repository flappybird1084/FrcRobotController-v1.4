package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final TalonFX shooterMotor = new TalonFX(24);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0.0);
    private final DoubleSupplier joystickInput;

    private final double maxRpm = 5000.0;
    private final double maxRps = maxRpm / 60.0;
    private final double deadband = 0.05;

    private double targetRps = 0.0;
    private double targetRpm = 0.0;
    private double currentRpm = 0.0;
    private double shooterPower = 0.0;
    private boolean enabled = false;

    public Shooter(DoubleSupplier joystickInput) {
        this.joystickInput = joystickInput;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 12.0 / maxRps;
        config.Slot0.kS = 0.2;

        shooterMotor.getConfigurator().apply(config);
    }

    public Command getDefaultCommand() {
        return run(this::runShooter);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            shooterMotor.set(0.0);
            targetRps = 0.0;
        }
    }

    public boolean atTargetSetpoint() {
        double currentRps = shooterMotor.getVelocity().getValueAsDouble();
        return Math.abs(currentRps - targetRps) <= 2.0;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getCurrentRpm() {
        return currentRpm;
    }

    public double getShooterPower() {
        return shooterPower;
    }

    private void runShooter() {
        if (!enabled) {
            shooterMotor.set(0.0);
            return;
        }
        double input = joystickInput.getAsDouble();
        if (Math.abs(input) < deadband) {
            input = 0.0;
        }

        targetRpm = input * maxRpm;
        targetRps = targetRpm / 60.0;
        currentRpm = shooterMotor.getVelocity().getValueAsDouble() * 60.0;
        shooterPower = shooterMotor.getDutyCycle().getValueAsDouble();

        if (input == 0.0) {
            shooterMotor.set(0.0);
        } else {
            shooterMotor.setControl(velocityControl.withVelocity(targetRps));
        }
    }
}
