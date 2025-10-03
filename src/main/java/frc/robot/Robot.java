// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  public static final CommandXboxController joystick = new CommandXboxController(0);

  public static double currentAngle;
  public static double targetAngle = RobotContainer.imu.getYaw().getValueAsDouble();


  double targetRotationalRate;

  public static final double JOYSTICK_YAW_MULTIPLIER = 4;

  //public static PIDController yawPIDController = new PIDController(0.02, 0.00003, 0.0015);
  // public static PIDController yawPIDController = new PIDController(0.0325, 0.00007, 0.002);
  public static PIDController yawPIDController = new PIDController(0.02, 0.0001, 0.00);

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    currentAngle = RobotContainer.imu.getYaw().getValueAsDouble();
    if(Math.abs(joystick.getRightX()) > 0.1){
      targetAngle -= joystick.getRightX()*JOYSTICK_YAW_MULTIPLIER;
    }
    
    yawPIDController.setSetpoint(targetAngle);
    targetRotationalRate = yawPIDController.calculate(currentAngle);

    RobotContainer.targetRotationalRate = targetRotationalRate;

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
