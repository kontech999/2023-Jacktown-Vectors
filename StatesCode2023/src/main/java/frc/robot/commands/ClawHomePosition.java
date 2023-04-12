// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ClawHomePosition extends CommandBase {
  /** Creates a new ClawHomePosition. */
  private double timerStart = 0;
  private double maxTime;
  private double abPos;
  public ClawHomePosition(double abPos, double maxTime) {
    this.maxTime = maxTime;
    this.abPos = abPos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timerStart = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This is home position
    // RobotContainer.m_arm.abMoveMotors(RobotContainer.m_arm.setABTicks(0.54, 0.3));
    // RobotContainer.m_arm.bcMoveMotors(RobotContainer.m_arm.setBCTicks(0.40, 0.3));
    RobotContainer.m_arm.moveArmMotors(-RobotContainer.m_arm.setABTicks(abPos, 0.8));
    //RobotContainer.m_arm.bcMoveMotors(RobotContainer.m_arm.setBCTicks(bcPos, 0.3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_arm.moveArmMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis()-timerStart>maxTime;
  }
}
