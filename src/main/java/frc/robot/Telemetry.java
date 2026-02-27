package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.AprilTags;
// import frc.robot.subsystems.ShooterBasic;
import frc.robot.subsystems.ShooterPID;

public class Telemetry {
    private final double MaxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
        SmartDashboard.putNumber("april tags detected", 0);
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* UDP telemetry */
    private final NetworkTable udpTable = inst.getTable("Udp");
    private final StringSubscriber udpLastPacket = udpTable.getStringTopic("LastPacket").subscribe("");
    private final DoubleSubscriber udpLastPacketBytes = udpTable.getDoubleTopic("LastPacketBytes").subscribe(0.0);
    private final DoubleSubscriber udpLastPacketTimestamp = udpTable.getDoubleTopic("LastPacketTimestamp").subscribe(0.0);

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];
    private final double[] m_moduleStatesArray = new double[8];
    private final double[] m_moduleTargetsArray = new double[8];

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        Pose2d pose = RobotContainer.driveSubsystem.getPose();
        drivePose.set(pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        m_poseArray[0] = pose.getX();
        m_poseArray[1] = pose.getY();
        m_poseArray[2] = pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            m_moduleStatesArray[i*2 + 0] = state.ModuleStates[i].angle.getRadians();
            m_moduleStatesArray[i*2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            m_moduleTargetsArray[i*2 + 0] = state.ModuleTargets[i].angle.getRadians();
            m_moduleTargetsArray[i*2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }

        SmartDashboard.putNumber("gyro angle", RobotContainer.imu.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("target angle", RobotContainer.driveSubsystem.targetAngle);
        SmartDashboard.putNumber("joystick x", RobotContainer.joystick.getRightX());
        SmartDashboard.putNumber("left trigger", RobotContainer.joystick.getLeftTriggerAxis());
        SmartDashboard.putNumber("ShooterTargetRPM", ShooterPID.targetRpm);
        SmartDashboard.putNumber("ShooterRPM", ShooterPID.currentRpm);
        SmartDashboard.putNumber("ShooterOutput", ShooterPID.shooterOutput);
        SmartDashboard.putNumber("ShooterFeederPower", ShooterPID.feederPower);
        SmartDashboard.putNumber("TagDistanceMeters", UdpTelemetryReceiver.getProcessorDistanceMeters());
        SmartDashboard.putString("udp last packet", udpLastPacket.get(""));
        SmartDashboard.putNumber("udp last timestamp", udpLastPacketTimestamp.get(0.0));
        SmartDashboard.putNumber("april tags detected", AprilTags.getDetectedCount());
        SmartDashboard.putNumber("single-tag robot offset x", UdpTelemetryReceiver.robotOffset.getX());
        SmartDashboard.putNumber("single-tag robot offset y", UdpTelemetryReceiver.robotOffset.getY());
        SmartDashboard.putNumber(
            "single-tag robot offset theta (deg)",
            UdpTelemetryReceiver.robotOffset.getRotation().getDegrees()
        );
        SmartDashboard.putNumber("vision robot pose x", UdpTelemetryReceiver.robotPose.getX());
        SmartDashboard.putNumber("vision robot pose y", UdpTelemetryReceiver.robotPose.getY());
        SmartDashboard.putNumber("processor delta x", UdpTelemetryReceiver.processorDelta.getX());
        SmartDashboard.putNumber("processor delta y", UdpTelemetryReceiver.processorDelta.getY());
        SmartDashboard.putNumber(
            "processor delta theta (deg)",
            Math.toDegrees(Math.atan2(
                UdpTelemetryReceiver.processorDelta.getY(),
                UdpTelemetryReceiver.processorDelta.getX()
            )) + UdpTelemetryReceiver.robotOffset.getRotation().getDegrees()
        );

        // SmartDashboard.putNumber("target pos", Elevator.getTargetPosition());
        // SmartDashboard.putNumber("current pos", Elevator.getCurrentPosition());
        // SmartDashboard.putNumber("power", Elevator.getPower());

        // SmartDashboard.putNumber("22 position", Elevator.elevator1.getEncoder().getPosition());
        // SmartDashboard.putNumber("23 position", Elevator.elevator2.getEncoder().getPosition());




        SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");
        fieldPub.set(m_poseArray);

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        }

        // SmartDashboard.putNumber("elevator1 position", RobotContainer.elevatorSubsystem.getElevatorPosition());
    }
}
