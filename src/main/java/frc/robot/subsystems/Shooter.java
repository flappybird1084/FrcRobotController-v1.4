package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Shooter extends SubsystemBase {
    CommandXboxController joystick;
    private static final PIDController pid = new PIDController(0.0002, 0.001, 0.000001);
    public static final double MAX_RPM = 5000.0;
    private static final double DEADBAND = 0.05;

    public static double targetRpm = 0.0;
    public static double currentRpm = 0.0;
    public static double shooterPower = 0.0;
    
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
            // periodic() handles PID control
        }, this);
    }

    public void periodic() {
        double input = joystick.getRightY();
        if (Math.abs(input) < DEADBAND) input = 0.0;

        targetRpm = input * MAX_RPM;

        currentRpm = encoder1.getVelocity(); // SparkMax RelativeEncoder uses RPM by default
        double output = pid.calculate(currentRpm, targetRpm);
        shooterPower = output; //for telemetry

        // clamp to [-1, 1]
        output = Math.max(-1.0, Math.min(1.0, output));
        shooter1.set(output);
    }
}
