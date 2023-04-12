// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class MoveTimeAuto extends CommandBase {
  /** Creates a new MoveTimeAuto. */
  double startTimer, power, time;

  public MoveTimeAuto(double power, double time) {
    // power equals the max speed
    this.power = power;
    // time equals how long you want this command to run in milliseconds
    this.time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // This tells you what time you started this command it is used to calculate time elapsed
    startTimer = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This make it so the value of speed goes from max speed to 0 when you finish the code
    // I divide by 1000 to make it usable for power
    double speed = (time-(System.currentTimeMillis()-startTimer))/1000;
    if(speed>power) {
      speed = power;
    }
    RobotContainer.m_chasis.moveLeft(speed);
    RobotContainer.m_chasis.moveRight(speed);
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
    // This is just saying if time elapsed is greater than max time finish the command
    return System.currentTimeMillis()-startTimer > time;
  }
}
