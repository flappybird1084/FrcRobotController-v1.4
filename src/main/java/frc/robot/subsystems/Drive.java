package frc.robot.subsystems;

import java.util.List;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;

public class Drive extends SubsystemBase {
    private final SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle();
    private final SwerveRequest.ApplyRobotSpeeds autoDrive = new SwerveRequest.ApplyRobotSpeeds();
    CommandSwerveDrivetrain drivetrain;
    double MaxSpeed = Constants.MaxSpeed;
    double scaling = Constants.scaling;
    CommandXboxController joystick;
    private Command lastPath;

    public double targetAngle;
    public static double kP; // 3.5
    public static double kI; // 0
    public static double kD; // 0.15
    private boolean aimAtTag;


    public Drive(CommandSwerveDrivetrain x, CommandXboxController joystick)
    {
        drivetrain = x;
        targetAngle = 0;
        this.joystick=joystick;
        kP = 3; // 5
        kI = 0.0;
        kD = 0.5;


        
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> driveRobotRelative(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(7.0, 0.0001, 0.01),
                        new PIDConstants(5.0, 0.0001, 0.045)
                ),
                config,
                () -> DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }

        
    }

    public void periodic(){
        double gyroAngle = Constants.imu.getYaw().getValueAsDouble();

        if (aimAtTag) {
            kP = 3.0;
            kI = 0.0001;
            kD = 0.15;
            return;
        }

        if(Math.abs(joystick.getRightX()) >= 0.08) {
            targetAngle += joystick.getRightX()*4;
            kP = 3.0;
            kI = 0.0001;
            kD = 0.15;
        } 

        // else if (Math.abs(gyroAngle-targetAngle) > 90) {
        //     kP = 2.0;
        // }
            
        else {
            kP = 0.0; // 0.5
            kI = 0.0; // 0.00001
            kD = 0.0; // 0.002
        }
    }
    
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

    public void aimAtTag(Pose2d robotOffset) {
        aimAtTag = true;
        double currentYaw = Constants.imu.getYaw().getValueAsDouble();
        targetAngle = currentYaw - robotOffset.getRotation().getDegrees();
    }

    public void setAimAtTagEnabled(boolean enabled) {
        aimAtTag = enabled;
        if (!enabled) {
            targetAngle = Constants.imu.getYaw().getValueAsDouble();
        }
    }

    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    public void cancelLastPath() {
        resetFacingAngle();
        if (lastPath != null) lastPath.cancel();
    }

    public void resetFacingAngle() {
        targetAngle = Math.toRadians(Constants.imu.getYaw().getValueAsDouble());
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return drivetrain.getState().Speeds;
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        drivetrain.setControl(autoDrive.withSpeeds(speeds));
    }

    // Autonomous
    public Command followPath(String pathName) {
        cancelLastPath();
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            path.preventFlipping = true;
            lastPath = AutoBuilder.followPath(path);
            return lastPath;
        } catch (IOException | ParseException e) {
            DriverStation.reportError("Failed to load path: " + pathName, e.getStackTrace());
            return Commands.none();
        }
    }


    public Command driveToPose(Pose2d endPose){
        Pose2d startPose = getPose();
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);
        Rotation2d endHeading = endPose.getRotation();

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new PathConstraints(
                0.5, 
                0.5,
                Units.degreesToRadians(0), 
                Units.degreesToRadians(0)
            ),
            null, // Ideal starting state can be null for on-the-fly paths
            new GoalEndState(0.0, endHeading) // Final heading matches endPos heading
        );

        path.preventFlipping = true;

        cancelLastPath();
        lastPath = AutoBuilder.followPath(path);
        
        return lastPath;
    }
    
    public Command pathRelative(double targetX, double targetY, double targetRotation) {
        System.out.println("path relative with target: " + targetX + ", " + targetY + ", " + targetRotation);
        System.out.println("current pose: " + getPose());
        
        Pose2d currentPose = getPose();

        Translation2d localOffset = new Translation2d(targetX, targetY);
        Translation2d fieldOffset = localOffset.rotateBy(currentPose.getRotation ());
        
        // Pose2d startPose = new Pose2d(
        //     currentPose.getTranslation(),
        //     currentPose.getRotation()
        // );

        Pose2d startPose = currentPose;

        Rotation2d endHeading = currentPose.getRotation().plus(new Rotation2d(targetRotation));

        Pose2d endPose = new Pose2d(
            currentPose.getTranslation().plus(fieldOffset),
            endHeading
        );
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);
        // PathPlannerPath path = new PathPlannerPath(
        //     waypoints,
        //     new PathConstraints(
        //         0.5, 
        //         0.5,
        //         Units.degreesToRadians(360), 
        //         Units.degreesToRadians(540)
        //     ),
        //     null, // Ideal starting state can be null for on-the-fly paths
        //     new GoalEndState(0.0, endHeading) // Final heading matches endPos heading
        // );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new PathConstraints(
                0.5, 
                0.5,
                Units.degreesToRadians(360), 
                Units.degreesToRadians(90)
            ),
            null, // Ideal starting state can be null for on-the-fly paths
            new GoalEndState(0.0, endHeading) // Final heading matches endPos heading
        );

        path.preventFlipping = true;

        cancelLastPath();
        lastPath = AutoBuilder.followPath(path);
        
        return lastPath;

    } 



}
