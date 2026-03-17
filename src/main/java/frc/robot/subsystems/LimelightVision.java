package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightVision extends SubsystemBase {
    private static final String LIMELIGHT_TABLE_NAME = "limelight";

    private final NetworkTable limelightTable =
        NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_NAME);

    private final DoublePublisher txPublisher =
        limelightTable.getDoubleTopic("RobotTx").publish();
    private final DoublePublisher tvPublisher =
        limelightTable.getDoubleTopic("RobotTv").publish();

    public double getTx() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    public boolean isTargetFound() {
        return limelightTable.getEntry("tv").getDouble(0.0) >= 1.0;
    }

    @Override
    public void periodic() {
        double tx = getTx();
        double tv = isTargetFound() ? 1.0 : 0.0;

        txPublisher.set(tx);
        tvPublisher.set(tv);

        SmartDashboard.putNumber("limelight tx", tx);
        SmartDashboard.putBoolean("limelight target found", tv >= 1.0);
    }
}
