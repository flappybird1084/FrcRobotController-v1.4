package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Intake extends SubsystemBase {
    private static final double INTAKE_POWER = 0.5;
    private static final double PIVOT_DOWN_POWER = 0.3;

    // Motors
    private final TalonFX intakeMotor = new TalonFX(27);
    private final TalonFX pivotMotor = new TalonFX(28);

    // Telemetry
    public static double intakePowerTelemetry = 0.0;

    public Intake(CommandXboxController joystick) {
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        var currentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true);
        pivotMotor.getConfigurator().apply(currentLimits);
    }

    /** Run intake in and push pivot down. */
    public void intake() {
        intakeMotor.set(INTAKE_POWER);
        pivotMotor.set(PIVOT_DOWN_POWER);
        intakePowerTelemetry = INTAKE_POWER;
    }

    /** Run intake out — pivot stays still. */
    public void outtake() {
        intakeMotor.set(-INTAKE_POWER);
        pivotMotor.set(0.0);
        intakePowerTelemetry = -INTAKE_POWER;
    }

    public void stop() {
        intakeMotor.set(0.0);
        pivotMotor.set(0.0);
        intakePowerTelemetry = 0.0;
    }

    public Command getDefaultCommand() {
        return new RunCommand(() -> {}, this);
    }
}
