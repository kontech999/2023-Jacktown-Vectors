// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class EncoderRotate extends CommandBase {
  private double startAngle;
  private double angle;
  private double motorSpeed;
  private boolean isFinished;
  private double startTime;
  private double maxTime;

  /** Creates a new EncoderRotate. */
  public EncoderRotate(double angle, double speed, double maxTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.motorSpeed = speed;
    this.maxTime = maxTime;
    addRequirements(RobotContainer.m_chasis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startAngle = Robot.m_robotContainer.getAngle();
    isFinished = false;
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = Robot.m_robotContainer.getAngle();
    double error = angle - (currentAngle - startAngle);
    double drivePower = Math.min(Constants.DRIVE_AUTO_ROTATE_KP * error, 1);
    SmartDashboard.putNumber("Angle", (currentAngle - startAngle));
    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("Drive Power", drivePower);
    SmartDashboard.putNumber("Time Elapsed", (System.currentTimeMillis() - startTime));
    SmartDashboard.putNumber("Max Time", maxTime);
    if (Math.abs(drivePower) > motorSpeed) {
      drivePower = Math.copySign(motorSpeed, drivePower);
    }
    RobotContainer.m_chasis.moveLeft(-drivePower);
    RobotContainer.m_chasis.moveRight(drivePower);

    if ((System.currentTimeMillis() - startTime) > maxTime) {
      RobotContainer.m_chasis.moveLeft(0);
      RobotContainer.m_chasis.moveRight(0);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_chasis.moveLeft(0);
    RobotContainer.m_chasis.moveRight(0);
  }

  public double inchesToTicks(double inches) {
    return inches * (21504 / (6 * Math.PI));
  }

  public double angleToTicks(double angle) {
    return inchesToTicks(angle * (22 * Math.PI) / 360);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
