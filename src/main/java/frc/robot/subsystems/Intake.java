package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
    private static final double INTAKE_STICK_SCALE = 0.30;
    private static final double PIVOT_MANUAL_SCALE = 0.35;
    private static final double JOYSTICK_DEADBAND = 0.05;

    private final TalonFX intakeMotor;
    private final TalonFX pivotMotor1;

    private final CommandXboxController joystick;

    public Intake(CommandXboxController joystick) {
        this.joystick = joystick;
        intakeMotor = new TalonFX(Constants.intakeMotorID);
        pivotMotor1 = new TalonFX(Constants.pivotMotorID);

        pivotMotor1.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        // Intake roller — right joystick Y
        double rightY = joystick.getRightY();
        if (Math.abs(rightY) > JOYSTICK_DEADBAND) {
            intakeMotor.set(-rightY * INTAKE_STICK_SCALE);
        } else {
            intakeMotor.set(0);
        }

        // Pivot — left joystick Y, direct power
        double leftY = joystick.getLeftY();
        if (Math.abs(leftY) > JOYSTICK_DEADBAND) {
            pivotMotor1.set(leftY * PIVOT_MANUAL_SCALE);
        } else {
            pivotMotor1.set(0);
        }

        SmartDashboard.putNumber("pivot/actual_pos", pivotMotor1.getPosition().getValueAsDouble());
    }
}
