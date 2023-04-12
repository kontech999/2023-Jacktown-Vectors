// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveTrain extends CommandBase {
  /** Creates a new DriveTrain. */

  public DriveTrain() {
    addRequirements(RobotContainer.m_chasis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_robotContainer.calibrateGyro();
    Robot.m_robotContainer.zeroGyro();
    RobotContainer.m_chasis.initializeMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftjoyY = Robot.m_robotContainer.GetleftjoyRawAxis(Constants.LEFT_JOYSTICK_Y_AXIS_ID);
    double rightjoyY = Robot.m_robotContainer.GetRightjoyRawAxis(Constants.RIGHT_JOYSTICK_Y_AXIS_ID);
    // These 2 if statments are used to stop drift
    if (Math.abs(leftjoyY) > 0.03) {
       RobotContainer.m_chasis.moveLeft(leftjoyY * leftjoyY * Math.signum(leftjoyY));
    } else {
      RobotContainer.m_chasis.moveLeft(0);
    }
    if (Math.abs(rightjoyY) > 0.03) {
       RobotContainer.m_chasis.moveRight(rightjoyY * rightjoyY * Math.signum(rightjoyY));
    } else {
      RobotContainer.m_chasis.moveRight(0);
    }

    // Debugging Print Statments
    SmartDashboard.putBoolean("Is Calibrating: ", Robot.m_robotContainer.isCalibrating());
    SmartDashboard.putBoolean("Is Connected: ", Robot.m_robotContainer.isConnected());
    SmartDashboard.putNumber("Yaw: ", Robot.m_robotContainer.getYaw());
    SmartDashboard.putNumber("Angle: ", Robot.m_robotContainer.getAngle());
    SmartDashboard.putNumber("Roll: ", Robot.m_robotContainer.getRoll());
    SmartDashboard.putNumber("Pitch: ", Robot.m_robotContainer.getPitch());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
