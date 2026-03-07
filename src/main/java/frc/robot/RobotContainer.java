// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

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
    private final UdpTelemetryReceiver udpTelemetryReceiver = new UdpTelemetryReceiver(Constants.udpTelemetryPort);

    public static final CommandXboxController joystick = new CommandXboxController(0);

    public static final CommandXboxController coJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        driveSubsystem = new Drive(drivetrain, joystick);
        shooterSubsystem = new Shooter(coJoystick);
        intakeSubsystem = new Intake(coJoystick);
        // elevatorSubsystem = new Elevator(coJoystick);
        // *** coralSubsystem = new Coral(coJoystick);
        udpTelemetryReceiver.start();
        configureBindings();
    }

    public double scaling = 0.3;

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        // drivetrain.setDefaultCommand(driveSubsystem.getDefaultCommand());
        driveSubsystem.useDefaultCommand();


        // shooterSubsystem.setDefaultCommand(shooterSubsystem.getDefaultCommand());

        // coJoystick.y().whileTrue(shooterSubsystem.runShooterCommand(coJoystick.getLeftY()));

        // coJoystick right bumper: auto-RPM from AprilTag distance (overrides manual while held)
        coJoystick.rightBumper().whileTrue(shooterSubsystem.autoRpmFromDistanceCommand());

        coJoystick.leftBumper().whileTrue(shooterSubsystem.runFeederMotors(0.5));
        coJoystick.y().whileTrue(shooterSubsystem.runFeederMotors(-0.5));
        coJoystick.leftBumper()
            .or(coJoystick.y())
            .whileFalse(shooterSubsystem.runFeederMotors(0));


        coJoystick.a().whileTrue(intakeSubsystem.runIntakeWithPivotDown(0.75));
        coJoystick.b().whileTrue(intakeSubsystem.runIntake(-1));


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

        joystick.rightBumper().onTrue(
            Commands.runOnce(() -> {
                // if (joystick.getRightTriggerAxis() > 0.5
                //     && UdpTelemetryReceiver.getSecondsSinceLastTag() > 0.4) {
                //     return;
                // }
                if (UdpTelemetryReceiver.isProcessorTagDetected()
                    && UdpTelemetryReceiver.isProcessorYawValid()
                    && udpTelemetryReceiver.getSecondsSinceLastTag() < 0.4) {
                    driveSubsystem.aimAtTag(UdpTelemetryReceiver.getProcessorYawError());
                } 
                // else {
                //     driveSubsystem.setAimAtTagEnabled(false);
                // }
            }, driveSubsystem)
        );
        joystick.rightBumper().onFalse(
            new InstantCommand(() -> driveSubsystem.setAimAtTagEnabled(false), driveSubsystem)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.x().onTrue(
                new InstantCommand(() -> {
                    driveSubsystem.pathRelative(1, 0, Math.toRadians(90)).schedule();
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
        final var autoDrive = new SwerveRequest.RobotCentric();

        return Commands.sequence(
            Commands.runOnce(() -> {
                // TODO: This maybe is off by 180?
                drivetrain.resetRotation(drivetrain.getOperatorForwardDirection());
            }),
            drivetrain.applyRequest(() ->
                autoDrive
                    .withVelocityX(-0.5 * MaxSpeed)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            ).withTimeout(3),
            drivetrain.applyRequest(() ->
                autoDrive
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            ).withTimeout(0.02)
        );
    }
}
