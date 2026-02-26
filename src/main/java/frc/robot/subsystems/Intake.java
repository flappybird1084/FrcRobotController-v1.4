package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Intake extends SubsystemBase {
    private final CommandXboxController joystick;

    // Pivot PID controller to hold position against gravity
    private final PIDController pivotPid = new PIDController(0.05, 0.0, 0.005);

    // Tunable gains
    public static double kP = 0.05;
    public static double kI = 0.0;
    public static double kD = 0.005;

    // Pivot target position (in encoder rotations)
    private double pivotTargetPosition = 0.0;

    // Joystick deadband and scaling
    private static final double DEADBAND = 0.05;
    private static final double PIVOT_SPEED = 0.5; // how fast joystick increments target
    private static final double INTAKE_POWER = 0.5;

    // Motors
    private final TalonFX intakeMotor = new TalonFX(27);
    private final TalonFX pivotMotor1 = new TalonFX(28);
    private final TalonFX pivotMotor2 = new TalonFX(29);

    // Telemetry values
    public static double currentPivotPosition = 0.0;
    public static double targetPivotPosition = 0.0;
    public static double pivotPower = 0.0;
    public static double intakePowerTelemetry = 0.0;

    public Intake(CommandXboxController joystick) {
        this.joystick = joystick;
        // Initialize target to current position so PID doesn't jump on startup
        pivotTargetPosition = pivotMotor1.getPosition().getValueAsDouble();
    }

    /**
     * Run the intake roller at the given power.
     * Positive = intake, negative = outtake.
     */
    public void setIntakePower(double power) {
        intakeMotor.set(power);
        intakePowerTelemetry = power;
    }

    /**
     * Stop the intake roller.
     */
    public void stopIntake() {
        intakeMotor.set(0.0);
        intakePowerTelemetry = 0.0;
    }

    /**
     * Set pivot motor power. Motor 2 runs opposite to motor 1.
     */
    private void setPivotPower(double power) {
        power = Math.max(-1.0, Math.min(1.0, power));
        pivotPower = power;
        pivotMotor1.set(power);
        pivotMotor2.set(-power); // always opposite direction
    }

    @Override
    public void periodic() {
        // Keep PID gains in sync for live tuning
        pivotPid.setP(kP);
        pivotPid.setI(kI);
        pivotPid.setD(kD);

        // Read left joystick Y for pivot control (up = negative Y in WPILib)
        double input = -joystick.getLeftY();
        if (Math.abs(input) < DEADBAND) input = 0.0;

        // Joystick increments the target position
        pivotTargetPosition += input * PIVOT_SPEED;

        // PID to hold pivot at target
        double currentPosition = pivotMotor1.getPosition().getValueAsDouble();
        double output = pivotPid.calculate(currentPosition, pivotTargetPosition);

        currentPivotPosition = currentPosition;
        targetPivotPosition = pivotTargetPosition;

        setPivotPower(output);
    }

    public Command getDefaultCommand() {
        return new RunCommand(() -> {
            // periodic() handles pivot PID control
        }, this);
    }

    public void resetPivotTarget() {
        pivotTargetPosition = pivotMotor1.getPosition().getValueAsDouble();
        pivotPid.reset();
    }
}
