package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
    // Motors
    private final TalonFX intakeMotor = new TalonFX(Constants.Intake.intakeMotorId);
    private final TalonFX pivotMotor = new TalonFX(Constants.Intake.pivotMotorId);

    // Telemetry
    public static double intakePowerTelemetry = 0.0;

    public Intake(CommandXboxController joystick) {
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        var currentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Intake.pivotStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true);
        pivotMotor.getConfigurator().apply(currentLimits);
    }

    /** Run intake in and push pivot down. */
    public void intake() {
        setMotorPowers(Constants.Intake.intakePower, Constants.Intake.pivotDownPower);
    }

    /** Run intake out — pivot stays still. */
    public void outtake() {
        setMotorPowers(-Constants.Intake.intakePower, 0.0);
    }

    public void stop() {
        setMotorPowers(0.0, 0.0);
    }

    private void setMotorPowers(double intakePower, double pivotPower) {
        intakeMotor.set(intakePower);
        pivotMotor.set(pivotPower);
        intakePowerTelemetry = intakePower;
    }

    public Command getDefaultCommand() {
        return new RunCommand(() -> {}, this);
    }
}
