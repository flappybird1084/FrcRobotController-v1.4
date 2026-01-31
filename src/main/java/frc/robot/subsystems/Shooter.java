package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Shooter extends SubsystemBase {
    CommandXboxController joystick;
    
    public Shooter(CommandXboxController joystick)
    {
        this.joystick=joystick;
    }

    public static final SparkMax shooter1 = new SparkMax(24, MotorType.kBrushless);
    
    public static final RelativeEncoder encoder1 = shooter1.getEncoder();

    public static void setCoralPower(double power) {
        shooter1.set(power); // Only 1 direction
    }

    public double getCoralPosition() {
        return encoder1.getPosition();
    }

    // Continuous movement
    public Command getDefaultCommand() {
        return new RunCommand(() -> {
            setCoralPower(joystick.getRightY());
        }, this);
    }
}