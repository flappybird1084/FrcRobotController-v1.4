package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Coral extends SubsystemBase {
    CommandXboxController joystick;
    
    public Coral(CommandXboxController joystick)
    {
        this.joystick=joystick;
    }

    public static final SparkMax coral1 = new SparkMax(24, MotorType.kBrushless);
    
    public static final RelativeEncoder encoder1 = coral1.getEncoder();


    public static void setCoralPower(double power) {
        coral1.set(power);
    }

    public double getCoralPosition() {
        return encoder1.getPosition();
    }

    public Command getDefaultCommand() {
       InstantCommand command = new InstantCommand(()->{
            setCoralPower(0.2 * (joystick.getRightY()));
       });
       command.addRequirements(this);
       return command;
    }
}