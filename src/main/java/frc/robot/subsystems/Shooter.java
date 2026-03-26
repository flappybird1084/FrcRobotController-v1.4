package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;

public class Shooter extends SubsystemBase {
    private final CommandXboxController joystick;

    public static final TalonFX blueMotor = new TalonFX(Constants.blueShooterMotorID);
    public static final TalonFX greenMotor = new TalonFX(Constants.greenShooterMotorID);
    public static final SparkMax feederMotor = new SparkMax(Constants.shooterFeederMotorID, MotorType.kBrushless);

    public static double currentRpm = 0.0;

    public Shooter(CommandXboxController joystick) {
        this.joystick = joystick;
    }

    @Override
    public void periodic() {
        double input = joystick.getLeftY();
        if (Math.abs(input) < Constants.shooterJoystickDeadband) {
            input = 0.0;
        }

        boolean bHeld = joystick.b().getAsBoolean();
        if (bHeld) {
            double presetOutput = Math.min(1.0, Math.abs(Constants.shooterPresetRPM) / Constants.shooterMaxRpm);
            blueMotor.set(presetOutput * Constants.blueShooterDirection);
            greenMotor.set(presetOutput * Constants.greenShooterDirection);
            feederMotor.set(Constants.shooterPresetFeederPower * Constants.feederDirection);
        } else {
            blueMotor.set(input * Constants.blueShooterDirection);
            greenMotor.set(input * Constants.greenShooterDirection);
            feederMotor.set(input * Constants.feederDirection);
        }

        double blueActualRpm = Math.abs(blueMotor.getVelocity().getValueAsDouble() * 60.0);
        double greenActualRpm = Math.abs(greenMotor.getVelocity().getValueAsDouble() * 60.0);
        currentRpm = (blueActualRpm + greenActualRpm) / 2.0;
    }
}
