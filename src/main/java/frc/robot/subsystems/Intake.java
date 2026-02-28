package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Intake extends SubsystemBase {
    private final CommandXboxController joystick;

    // Hardware PID position control (runs at 1kHz on the Kraken)
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    // Pivot target position (in encoder rotations)
    private double pivotTargetPosition = 0.0;

    // Joystick deadband and scaling
    private static final double DEADBAND = 0.05;
    private static final double PIVOT_SPEED = 0.5;
    private static final double INTAKE_POWER = 0.5;

    // Motors
    private final TalonFX intakeMotor = new TalonFX(27);
    private final TalonFX pivotMotor1 = new TalonFX(28);

    // Telemetry values
    public static double currentPivotPosition = 0.0;
    public static double targetPivotPosition = 0.0;
    public static double intakePowerTelemetry = 0.0;

    public Intake(CommandXboxController joystick) {
        this.joystick = joystick;

        // Brake mode so the pivot doesn't fall when power is cut
        pivotMotor1.setNeutralMode(NeutralModeValue.Brake);

        // Hardware PID gains on motor controller (1kHz loop)
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 2.0;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.1;
        // slot0Configs.kG = 0.5; // Uncomment and tune to counter gravity

        pivotMotor1.getConfigurator().apply(slot0Configs);

        // Stator current limit to protect motors from stalling under load
        var currentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true);
        pivotMotor1.getConfigurator().apply(currentLimits);

        // Initialize target to current position so it doesn't jump on startup
        pivotTargetPosition = pivotMotor1.getPosition().getValueAsDouble();
    }

    public void setIntakePower(double power) {
        intakeMotor.set(power);
        intakePowerTelemetry = power;
    }

    public void stopIntake() {
        intakeMotor.set(0.0);
        intakePowerTelemetry = 0.0;
    }

    @Override
    public void periodic() {
        // Read left joystick Y for pivot control (up = negative Y in WPILib)
        double input = -joystick.getLeftY();
        if (Math.abs(input) < DEADBAND) input = 0.0;

        // Joystick increments the target position
        pivotTargetPosition += input * PIVOT_SPEED;

        // Send target to motor — Kraken holds position at 1kHz internally
        pivotMotor1.setControl(positionRequest.withPosition(pivotTargetPosition));

        // Telemetry
        currentPivotPosition = pivotMotor1.getPosition().getValueAsDouble();
        targetPivotPosition = pivotTargetPosition;
    }

    public Command getDefaultCommand() {
        return new RunCommand(() -> {
            // periodic() handles pivot control
        }, this);
    }

    public void resetPivotTarget() {
        pivotTargetPosition = pivotMotor1.getPosition().getValueAsDouble();
    }
}
