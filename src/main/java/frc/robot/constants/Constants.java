package frc.robot.constants;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.constants.TunerConstants;

public class Constants {
    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double scaling = 0.3;

    public static final int pigeonID = 15;
    public static final Pigeon2 imu = new Pigeon2(Constants.pigeonID);

    public static final int udpTelemetryPort = 5800;

    public static final double shooterMaxRps = 100.0;
    public static final double shooterKp = 0.11;
    public static final double shooterKi = 0.0;
    public static final double shooterKd = 0.0;
    public static final double shooterKv = 12.0 / shooterMaxRps;
}
