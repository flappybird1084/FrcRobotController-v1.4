package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;

public class Drive extends SubsystemBase {
    private final SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle();
    CommandSwerveDrivetrain drivetrain;
    double MaxSpeed = Constants.MaxSpeed;
    double scaling = Constants.scaling;
    CommandXboxController joystick;

    public double targetAngle;
    public static double kP; // 3.5
    public static double kI; // 0
    public static double kD; // 0.15

    public Drive(CommandSwerveDrivetrain x, CommandXboxController joystick)
    {
        drivetrain = x;
        targetAngle = 0;
        this.joystick=joystick;
        kP = 3; // 5
        kI = 0.0;
        kD = 0.5;
    }

    public void periodic(){
        if(Math.abs(joystick.getRightX()) >= 0.06) {
            targetAngle += joystick.getRightX()*4;
            kP = 3.0;
            kI = 0.0001;
            kD = 0.15;
        } else {
            kP = 0.0;
            kI = 0.0; // 0.001
            kD = 0.0; // 0.03
        }
    }

    /* public void periodic(){
        double rightX = joystick.getRightX();
        if(Math.abs(rightX) >= 0.06) {
            targetAngle += joystick.getRightX()*4;
        }

        kP = 2.0;
        kI = 0.0001;
        kD = 0.25;
    } */
    
    public Command getDefaultCommand() {
        // todo int scaling = 0.3;
        return drivetrain.applyRequest(() ->
                 drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * scaling) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * scaling) // Drive left with negative X (left)
                    .withTargetDirection(new Rotation2d(-Math.toRadians(targetAngle))) // Drive counterclockwise with negative X (left)
                    .withHeadingPID(kP, kI, kD)
                    );
    }

    public void useDefaultCommand(){
        drivetrain.setDefaultCommand(getDefaultCommand());
    }

    public void resetTargetAngle(double angle){
        targetAngle = angle;
    }

}
