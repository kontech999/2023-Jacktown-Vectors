// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SlideAuto extends CommandBase {
  /** Creates a new SlideAuto. */
  private double timerStart = 0;
  private double maxTime;
  private double bcPos;

  public SlideAuto(double bcPos, double maxTime) {
    this.maxTime = maxTime;
    this.bcPos = bcPos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timerStart = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This make sure that in auto the sled never moves past the limit switch
    if (RobotContainer.m_arm.getExtendState()) {
      RobotContainer.m_arm.moveSledMotors(-RobotContainer.m_arm.setBCTicks(bcPos, 0.8));
    } else {
      RobotContainer.m_arm.moveSledMotors(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_arm.moveSledMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This represents how much time elapsed and if time elapsed is greater than time limit stop comand
    return System.currentTimeMillis() - timerStart > maxTime;
  }
}
