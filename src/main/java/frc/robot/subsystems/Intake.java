package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
    private static final double INTAKE_STICK_SCALE = 0.60;
    private static final double JOYSTICK_DEADBAND  = 0.05;

    private static final double PIVOT_UP_POS   = -2.115;
    private static final double PIVOT_DOWN_POS = -0.1;

    private final TalonFX intakeMotor;
    private final TalonFX pivotMotor1;
    private final PositionVoltage pivotPositionRequest = new PositionVoltage(0).withSlot(0);

    private final CommandXboxController joystick;

    // Auto override: when active, periodic() uses this power instead of joystick
    private boolean autoOverrideActive = false;
    private double autoOverridePower = 0.0;

    // Snap-to-position: when active, periodic() holds pivotSnapTarget via position PID
    private boolean pivotSnapActive = false;
    private double pivotSnapTarget = 0.0;

    public Intake(CommandXboxController joystick) {
        this.joystick = joystick;
        intakeMotor = new TalonFX(Constants.intakeMotorID);
        pivotMotor1 = new TalonFX(Constants.pivotMotorID);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.Slot0.kP = 5.0;
        pivotConfig.Slot0.kD = 0.2;
        pivotConfig.Slot0.kG = 0.3;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotMotor1.getConfigurator().apply(pivotConfig);

        pivotMotor1.setNeutralMode(NeutralModeValue.Brake);
    }

    /** Snap pivot to fully up position (-2.115). Bind with onTrue(). */
    public Command snapPivotUpCommand() {
        return Commands.runOnce(() -> { pivotSnapActive = true; pivotSnapTarget = PIVOT_UP_POS; }, this);
    }

    /** Snap pivot to fully down position (0). Bind with onTrue(). */
    public Command snapPivotDownCommand() {
        return Commands.runOnce(() -> { pivotSnapActive = true; pivotSnapTarget = PIVOT_DOWN_POS; }, this);
    }

    /** Zeroes the pivot encoder position. Bind with onTrue(). */
    public Command resetPivotEncoderCommand() {
        return Commands.runOnce(() -> pivotMotor1.setPosition(0), this);
    }

    /** Run the intake roller at a fixed power from an auto command, bypassing joystick. */
    public Command runIntakeAutoCommand(double power) {
        return Commands.startEnd(
            () -> { autoOverrideActive = true; autoOverridePower = power; },
            () -> { autoOverrideActive = false; autoOverridePower = 0.0; },
            this
        );
    }

    @Override
    public void periodic() {
        if (autoOverrideActive) {
            intakeMotor.set(autoOverridePower);
            double actualPos = pivotMotor1.getPosition().getValueAsDouble();
            SmartDashboard.putNumber("pivot/actual_pos", actualPos);
            return;
        }

        // Intake roller — right joystick Y
        double rightY = joystick.getRightY();
        if (Math.abs(rightY) > JOYSTICK_DEADBAND) {
            intakeMotor.set(-rightY * INTAKE_STICK_SCALE);
        } else {
            intakeMotor.set(0);
        }

        // Pivot — hold snap target if active, otherwise brake
        if (pivotSnapActive) {
            pivotMotor1.setControl(pivotPositionRequest.withPosition(pivotSnapTarget));
        } else {
            pivotMotor1.set(0);
        }

        double actualPos = pivotMotor1.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("pivot/actual_pos", actualPos);
    }
}
