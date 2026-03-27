package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;

public class Shooter extends SubsystemBase {
    private final CommandXboxController joystick;

    public static double currentRpm = 0.0;       // blue (lower) motor RPM
    public static double currentGreenRpm = 0.0;  // green (upper) motor RPM

    public enum ShootingMode { MEDIUM, FAR }
    static ShootingMode shootingMode = ShootingMode.MEDIUM;  // package-visible for Telemetry

    private boolean presetActive = false;
    private boolean presetFeedsEnabled = true;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(1);

    private static final TalonFX blueMotor  = new TalonFX(Constants.blueShooterMotorID);
    private static final TalonFX greenMotor = new TalonFX(Constants.greenShooterMotorID);
    private static final SparkMax feeder    = new SparkMax(Constants.shooterFeederMotor1ID, MotorType.kBrushless);

    public Shooter(CommandXboxController joystick) {
        this.joystick = joystick;

        // Green spins opposite to blue so both wheels propel the game piece the same way
        TalonFXConfiguration greenConfig = new TalonFXConfiguration();
        greenConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        greenMotor.getConfigurator().apply(greenConfig);

        // Slot 1: velocity closed-loop gains used by preset mode
        Slot1Configs slot1 = new Slot1Configs();
        slot1.kV = Constants.shooterKv;
        slot1.kP = Constants.shooterKp;
        blueMotor.getConfigurator().apply(slot1);
        greenMotor.getConfigurator().apply(slot1);
    }

    /** Set shooting mode (MEDIUM or FAR). */
    public static void setShootingMode(ShootingMode mode) {
        shootingMode = mode;
    }

    /** Reset to competition defaults (called on teleop init). */
    public static void resetToDefaults() {
        shootingMode = ShootingMode.MEDIUM;
    }

    /**
     * Hold this command to run flywheels + feeder at the currently selected preset RPM.
     * Releasing returns control to left-Y joystick.
     */
    public Command runShooterAtPresetCommand() {
        return Commands.startEnd(
            () -> {
                presetActive = true;
                presetFeedsEnabled = true;
            },
            () -> {
                presetActive = false;
                feeder.set(0);
            },
            this
        );
    }

    /**
     * Hold this command to run flywheels at preset RPM without running the feeder.
     * Releasing returns control to left-Y joystick.
     */
    public Command warmupShooterAtPresetCommand() {
        return Commands.startEnd(
            () -> {
                presetActive = true;
                presetFeedsEnabled = false;
            },
            () -> {
                presetActive = false;
                feeder.set(0);
            },
            this
        );
    }

    public static void setFeederPower(double power) {
        feeder.set(-power);
    }

    public Command runFeederMotors(double power) {
        return new RunCommand(() -> setFeederPower(power), this);
    }

    @Override
    public void periodic() {
        if (presetActive) {
            double blueTargetRpm, greenTargetRpm;
            if (shootingMode == ShootingMode.FAR) {
                blueTargetRpm  = Constants.closeShooterBlueRpm;
                greenTargetRpm = Constants.closeShooterGreenRpm;
            } else { // MEDIUM
                blueTargetRpm  = Constants.mediumShooterRpm;
                greenTargetRpm = Constants.mediumShooterRpm;
            }
            blueMotor.setControl(velocityRequest.withVelocity(blueTargetRpm / 60.0));
            greenMotor.setControl(velocityRequest.withVelocity(greenTargetRpm / 60.0));
            boolean atSpeed = currentRpm >= 0.9 * blueTargetRpm;
            feeder.set((presetFeedsEnabled && atSpeed) ? Constants.presetFeederPower : 0.0);
        } else {
            double input = joystick.getLeftY();
            blueMotor.set(0.9*input);
            greenMotor.set(0.9*input);
            feeder.set(input);  // feeder runs with shooter via left Y
        }
        currentRpm      = blueMotor.getVelocity().getValueAsDouble() * 60.0;
        currentGreenRpm = greenMotor.getVelocity().getValueAsDouble() * 60.0;
    }
}
