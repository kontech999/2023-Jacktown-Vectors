// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class limelightCommand extends CommandBase {
  /** Creates a new limelight. */
  public limelightCommand() {
    addRequirements(RobotContainer.m_chasis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This tells us if the limelight detecting anything; 0 if no, 1 if yes
    double targetValid = Robot.m_robotContainer.getIsTargetValid();
    // Gets the x-axis based on the center of the camera
    double x = Robot.m_robotContainer.getAngleX() - 2;
    // Gets the y-axis based on the center of the camera
    double y = Robot.m_robotContainer.getAngleY();
    // Tells how much the target is taking up the screen from 0 - 100%
    double area = Robot.m_robotContainer.getArea();

    // Tells us values
    SmartDashboard.putNumber("Target Valid", targetValid);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    // targetValid will always either be 0 or 1 so I just pick 0.8 as a random
    // number
    // Degrees threshold tells us how off we can be from alignment
    if (targetValid > 0.8 && Math.abs(x) > Constants.DEGREES_THRESHOLD) {
      RobotContainer.m_chasis.moveLeft(-x * Constants.LIMELIGHT_P);
      RobotContainer.m_chasis.moveRight(x * Constants.LIMELIGHT_P);
    }else {
      RobotContainer.m_chasis.moveLeft(0);
      RobotContainer.m_chasis.moveRight(0);
    }
    
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
