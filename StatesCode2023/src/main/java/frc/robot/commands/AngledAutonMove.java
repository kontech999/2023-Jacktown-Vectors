// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* This Command is used for going over the charge station in a smooth manner so that it doesnt slip and slide */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AngledAutonMove extends CommandBase {
  private double currentAngle;
  double anglewhenstop;
  // This Boolean variable represent if we are going down or up on the charge station
  boolean highangleorlowangle;
  /** Creates a new AngledAutonMove. */
  public AngledAutonMove(double anglewhenstop, boolean highangleorlowangle) {
    this.anglewhenstop = anglewhenstop; // Decide what angle you want the code to stop
    this.highangleorlowangle = highangleorlowangle; // Tells if your going up or down the ramp
    addRequirements(RobotContainer.m_chasis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This just updates the angle value
    this.currentAngle = (Robot.m_robotContainer.getPitch() - Constants.NAVX_OFFSET);
    
    RobotContainer.m_chasis.moveLeft(0.2);
    RobotContainer.m_chasis.moveRight(0.2);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_chasis.moveLeft(0);
    RobotContainer.m_chasis.moveRight(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (highangleorlowangle){
      return Math.abs(currentAngle) < anglewhenstop;
    }
    // highangleorlowangle isnt used anymore so only the code below will run
    return currentAngle < anglewhenstop;
  }
}
