package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Intake extends SubsystemBase {
    private static final double INTAKE_TRIGGER_SCALE = 0.70;
    private static final double PIVOT_MANUAL_SCALE = 0.50;
    private static final double PIVOT_DOWN_POWER = 0.30;

    // Motors
    private final TalonFX intakeMotor;
    private final TalonFX pivotMotor1;

    private final CommandXboxController joystick;

    private boolean intakeOverrideActive = false;
    private double intakeOverridePower = 0.0;
    private boolean forcePivotDown = false;

     // Reusable control request — mutated each loop via withPosition() if we do position control.
    public Intake(CommandXboxController joystick) {
        this.joystick = joystick;
        intakeMotor = new TalonFX(26);
        pivotMotor1 = new TalonFX(25);

        // Brake mode so the pivot doesn't fall when power is cut
        pivotMotor1.setNeutralMode(NeutralModeValue.Brake);

    }

    public void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    public void setPivotPower(double power){
        pivotMotor1.set(power);
    }

    public Command runIntake(double power){
        return Commands.startEnd(
            () -> setIntakeOverride(power),
            this::clearIntakeOverride,
            this
        );
    }

    public Command runPivot(double power){
        return new RunCommand(()->{
            setPivotPower(power);
        }, this);
    }

    public Command runIntakeWithPivotDown(double power) {
        return Commands.startEnd(
            () -> {
                
                setIntakeOverride(power);
                forcePivotDown = true;
            },
            () -> {
                clearIntakeOverride();
                forcePivotDown = false;
            },
            this
        );
    }

    @Override
    public void periodic() {
        // runIntake(joystick.getLeftTriggerAxis());
        double intakePower = intakeOverrideActive
            ? intakeOverridePower
            : INTAKE_TRIGGER_SCALE * (joystick.getLeftTriggerAxis() - joystick.getRightTriggerAxis());
        intakeMotor.set(intakePower*0.74
        
        
        );

        double pivotPower = forcePivotDown
            ? PIVOT_DOWN_POWER
            : PIVOT_MANUAL_SCALE * (joystick.getLeftY());
        pivotMotor1.set(pivotPower);
        // Telemetry or sensor reading can be added here if needed
    }

    // @Override
    // public void periodic() {
    //     // Read left joystick Y for pivot control (up = negative Y in WPILib)
    //     double input = -joystick.getLeftY();
    //     if (Math.abs(input) < DEADBAND) input = 0.0;

    //     // Joystick increments the target position
    //     pivotTargetPosition += input * PIVOT_SPEED;

    //     // Send target to motor — Kraken holds position at 1kHz internally
    //     pivotMotor1.setControl(positionRequest.withPosition(pivotTargetPosition));

    //     // Telemetry
    //     currentPivotPosition = pivotMotor1.getPosition().getValueAsDouble();
    //     targetPivotPosition = pivotTargetPosition;
    // }

    // public Command getDefaultCommand() {
    //     return new RunCommand(() -> {
    //         // periodic() handles pivot control
    //     }, this);
    // }

    // public void resetPivotTarget() {
    //     pivotTargetPosition = pivotMotor1.getPosition().getValueAsDouble();
    // }

    private void setIntakeOverride(double power) {
        intakeOverrideActive = true;
        intakeOverridePower = power;
    }

    private void clearIntakeOverride() {
        intakeOverrideActive = false;
    }

}
