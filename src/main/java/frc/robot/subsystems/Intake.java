package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // Motors
    private final TalonFX intakeMotor;
    private final TalonFX pivotMotor1;

   

    public Intake() {
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
        return new RunCommand(()->{
            setIntakePower(power);
        }, this);
    }

    public Command runPivot(double power){
        return new RunCommand(()->{
            setPivotPower(power);
        }, this);
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


}
