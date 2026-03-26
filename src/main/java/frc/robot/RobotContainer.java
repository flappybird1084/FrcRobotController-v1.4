// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public static Drive driveSubsystem;
    // public static Elevator elevatorSubsystem;
    // public static Coral coralSubsystem;
    public static Shooter shooterSubsystem;
    public static Intake intakeSubsystem;
    
    public static final Pigeon2 imu = new Pigeon2(Constants.pigeonID);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    // private final UdpTelemetryReceiver udpTelemetryReceiver = new UdpTelemetryReceiver(Constants.udpTelemetryPort); // APRILTAG/CAMERA DISABLED FOR COMP 2
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public static final CommandXboxController joystick = new CommandXboxController(0);

    public static final CommandXboxController coJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        driveSubsystem = new Drive(drivetrain, joystick);
        shooterSubsystem = new Shooter(coJoystick);
        intakeSubsystem = new Intake(coJoystick);
        // elevatorSubsystem = new Elevator(coJoystick);
        // *** coralSubsystem = new Coral(coJoystick);
        // udpTelemetryReceiver.start(); // APRILTAG/CAMERA DISABLED FOR COMP 2
        configureBindings();
        configureAutoChooser();
    }

    public double scaling = 0.3;

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        // drivetrain.setDefaultCommand(driveSubsystem.getDefaultCommand());
        driveSubsystem.useDefaultCommand();


        coJoystick.rightTrigger().onTrue(intakeSubsystem.resetPivotEncoderCommand());
        coJoystick.x().onTrue(intakeSubsystem.snapPivotUpCommand());
        coJoystick.a().onTrue(intakeSubsystem.snapPivotDownCommand());




        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * scaling) // Drive forward with negative Y (forward)
        //             .withVelocityY(-joystick.getLeftX() * MaxSpeed * scaling) // Drive left with negative X (left)
        //             .withTargetDirection(dd) // Drive counterclockwise with negative X (left)
        //             .withHeadingPID(1,0,0)
        //     )
        // );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // APRILTAG/CAMERA AIM DISABLED FOR COMP 2.
        // joystick.rightBumper().onTrue(...)
        // joystick.rightBumper().onFalse(...)

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.x().and(joystick.back().negate()).and(joystick.start().negate()).onTrue(
                new InstantCommand(() -> {
                    driveSubsystem.compBackoffCommand().schedule(); // CHANGE THIS AT PRACTICE via Constants.compBackoffMeters.
                }, driveSubsystem
        ));

        joystick.y().onTrue(new InstantCommand(() -> driveSubsystem.resetPose(new Pose2d()), driveSubsystem));
        
        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        .andThen(new InstantCommand(()->
            {driveSubsystem.resetTargetAngle(0);}
        ))
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureAutoChooser() {
        autoChooser.addOption("Do Nothing", Commands.none());
        autoChooser.addOption("Path Left_Ball_Scrape", buildLeftBallPathAuto());
        autoChooser.addOption("Path Right_Ball_Scrape", buildRightBallPathAuto());
        autoChooser.addOption("Path Mid_Left", buildMidLeftPathAuto());
        autoChooser.addOption("Path Mid_Right", buildMidRightPathAuto());
        autoChooser.addOption("Path Move_Back", buildBackPathAuto());
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // All Autonomous paths
    private Command buildLeftBallPathAuto() { // Starts left trench
        // ── Intake timing (tune these to match actual path duration) ──────────────
        // Estimated total path time: ~12-15 s. Start intake at ~40%, stop at ~80%.
        final double INTAKE_START_DELAY = 5.0;  // seconds before intake turns on
        final double INTAKE_DURATION    = 5.0;  // seconds to run intake
        final double INTAKE_POWER       = 0.75;
        // ─────────────────────────────────────────────────────────────────────────

        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("leftballpickup");
            return Commands.sequence(
                Commands.runOnce(() ->
                    drivetrain.resetPose(path.getStartingHolonomicPose().orElse(new Pose2d()))
                ),
                Commands.parallel(
                    AutoBuilder.followPath(path).deadlineFor(drivetrain.run(() -> {})),
                    Commands.sequence(
                        Commands.waitSeconds(INTAKE_START_DELAY),
                        intakeSubsystem.runIntakeAutoCommand(INTAKE_POWER)
                            .withTimeout(INTAKE_DURATION)
                    )
                )
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load path 'd': " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    private Command buildRightBallPathAuto() { // Starts right trench
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("rightballpickup");
            return Commands.sequence(
                Commands.runOnce(() ->
                    drivetrain.resetPose(path.getStartingHolonomicPose().orElse(new Pose2d()))
                ),
                AutoBuilder.followPath(path).deadlineFor(drivetrain.run(() -> {}))
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load path 'b': " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    private Command buildMidLeftPathAuto() { // Starts middle, goes to left trench
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("midleftcarry");
            return Commands.sequence(
                Commands.runOnce(() ->
                    drivetrain.resetPose(path.getStartingHolonomicPose().orElse(new Pose2d()))
                ),
                AutoBuilder.followPath(path).deadlineFor(drivetrain.run(() -> {}))
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load path 'b': " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    private Command buildMidRightPathAuto() { // Starts middle, goes to right trench
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("midrightcarry");
            return Commands.sequence(
                Commands.runOnce(() ->
                    drivetrain.resetPose(path.getStartingHolonomicPose().orElse(new Pose2d()))
                ),
                AutoBuilder.followPath(path).deadlineFor(drivetrain.run(() -> {}))
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load path 'b': " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    private Command buildBackPathAuto() { // Starts left trench
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("moveback");
            return Commands.sequence(
                Commands.runOnce(() ->
                    drivetrain.resetPose(path.getStartingHolonomicPose().orElse(new Pose2d()))
                ),
                AutoBuilder.followPath(path).deadlineFor(drivetrain.run(() -> {}))
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load path 'd': " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}
