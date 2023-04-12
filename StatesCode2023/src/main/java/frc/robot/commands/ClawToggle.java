// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ClawToggle extends CommandBase {
  private boolean solenoidClawOut = true;
  /** Creates a new ClawToggle. */
  public ClawToggle() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(solenoidClawOut) {
      RobotContainer.m_claw.ClawPneuOut();
      SmartDashboard.putBoolean("Claw", true);
    }else {
      RobotContainer.m_claw.ClawPneuIn();
      SmartDashboard.putBoolean("Claw", false);
    }
    solenoidClawOut = !solenoidClawOut;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
