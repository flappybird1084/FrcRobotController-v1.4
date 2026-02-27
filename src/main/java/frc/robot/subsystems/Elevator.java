package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Elevator extends SubsystemBase {
    CommandXboxController joystick;
    public Elevator(CommandXboxController joystick)
    {
        this.joystick=joystick;
        // initialize target to current position so PID doesn't try to move immediately
        targetPosition = getElevatorPosition();
    }

    public static final SparkMax elevator1 = new SparkMax(22, MotorType.kBrushless);
    public static final SparkMax elevator2 = new SparkMax(23, MotorType.kBrushless);
    
    public static final RelativeEncoder encoder1 = elevator1.getEncoder();
    public static final RelativeEncoder encoder2 = elevator2.getEncoder();

    // PID controller for holding position
    private static final PIDController pid = new PIDController(0.8, 0.0, 0.05);

    // Tunable gains (updated from periodic like Drive.java)
    public static double kP = 0.8;
    public static double kI = 0.0;
    public static double kD = 0.05;

    // Desired target position (in encoder units)
    private static double targetPosition = encoder1.getPosition();

    private static double p;

    private static double x = 0.3;

    public static void setElevatorPower(double power) {
        // clamp to [-1, 1]k
        p = Math.max(-1.0, Math.min(1.0, power));
        elevator1.set(x*p);
        elevator2.set(x*p);
    }

    public double getElevatorPosition() {
        return encoder1.getPosition();
    }

    public void periodic() {
        // Keep PID gains in sync (allows tuning like Drive.java)
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);

        // Read manual input (left trigger up, right trigger down)
        double manual = joystick.getLeftTriggerAxis() - joystick.getRightTriggerAxis();

        targetPosition += manual;
        double currentposition = encoder1.getPosition();
        double power = pid.calculate(currentposition, targetPosition);

        setElevatorPower(power);
        setElevatorPower(manual);
    }

    public static double getTargetPosition() {
        return targetPosition;
    }

    public static double getCurrentPosition() {
        return encoder1.getPosition();
    }

    public static double getPower() {
        return p;
    }
    
    public Command getDefaultCommand() {
        // Default command is a no-op; periodic() handles manual and PID control.
        return new edu.wpi.first.wpilibj2.command.RunCommand(
            () -> {},
            this
        );
    }

    public void resetTargetPosition(double position) {
        targetPosition = position;
        pid.reset();
    }
}