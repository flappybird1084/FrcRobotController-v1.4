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

    // Existing telemetry compatibility fields.
    public static double targetRpm = 0.0;
    public static double currentRpm = 0.0;
    public static double shooterPower = 0.0;

    // Added telemetry fields for fast pit tuning.
    public static double leftYInput = 0.0;
    public static double blueCmdPower = 0.0;
    public static double greenCmdPower = 0.0;
    public static double feederCmdPower = 0.0;
    public static double blueActualRpm = 0.0;
    public static double greenActualRpm = 0.0;

    public Shooter(CommandXboxController joystick) {
        this.joystick = joystick;
    }

    @Override
    public void periodic() {
        double input = joystick.getLeftY();
        if (Math.abs(input) < Constants.shooterJoystickDeadband) {
            input = 0.0;
        }

        leftYInput = input;
        blueCmdPower = input * Constants.shooterStickScale * Constants.blueShooterDirection;
        greenCmdPower = input * Constants.shooterStickScale * Constants.greenShooterDirection;
        feederCmdPower = input * Constants.feederStickScale * Constants.feederDirection;

        blueMotor.set(blueCmdPower);
        greenMotor.set(greenCmdPower);
        feederMotor.set(feederCmdPower);

        blueActualRpm = blueMotor.getVelocity().getValueAsDouble() * 60.0;
        greenActualRpm = greenMotor.getVelocity().getValueAsDouble() * 60.0;
        targetRpm = blueCmdPower * Constants.shooterMaxRpm;
        currentRpm = blueActualRpm;
        shooterPower = blueCmdPower;
    }
}
