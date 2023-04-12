// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class PathPlannerLeft extends CommandBase {
  /** Creates a new PathPlannerLeft. */
  double timerStart;
  PathPlannerTrajectory firstPath = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));
  public PathPlannerLeft() {
    addRequirements(RobotContainer.m_chasis);
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
    PathPlannerState state = (PathPlannerState) firstPath.sample((System.currentTimeMillis()-timerStart)/1000);
    double rotation = state.angularVelocityRadPerSec*0.0508;
    double leftSpeed = state.velocityMetersPerSecond-rotation;
    double rightSpeed = state.velocityMetersPerSecond+rotation;
    //normalize(rightSpeed, leftSpeed, state.velocityMetersPerSecond);
    RobotContainer.m_chasis.moveLeft(leftSpeed*0.025);
    RobotContainer.m_chasis.moveRight(rightSpeed*0.025);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis()-timerStart>20000;
  }
}
