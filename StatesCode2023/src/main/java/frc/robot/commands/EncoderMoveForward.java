// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class EncoderMoveForward extends CommandBase {
  private boolean isFinished = false;
  private double distance;
  private double motorSpeed;
  private double[] motorPos = new double[4];
  private double maxSpeed;
  private double tickThreshold = 50;
  private double startTime;
  private double maxTime;

  /** Creates a new EncoderMoveForward. */
  public EncoderMoveForward(double distance, double motorSpeed, double maxTime) {
    this.distance = inchesToTicks(distance);
    this.motorSpeed = motorSpeed;
    this.maxTime = maxTime;
    addRequirements(RobotContainer.m_chasis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_chasis.updateStartPoints();// This tells you what position the encoders are in each motors when you start the command
    isFinished = false;
    maxSpeed = 0.05;//This is a varible respresent the current max speed this value will increase as time goes on
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motorPos = RobotContainer.m_chasis.getMotorPosition();
    SmartDashboard.putNumber("Relative Position", (motorPos[1] - RobotContainer.m_chasis.startPoints[1])*-1);
    SmartDashboard.putBoolean("Reach Point",(motorPos[1] - RobotContainer.m_chasis.startPoints[1]*-1) > distance);
    SmartDashboard.putNumber("distance", distance);
    SmartDashboard.putNumber("Time Elapsed", System.currentTimeMillis()-startTime);

 
    if (!isFinished) {
      // error1 and drivepower1 are for the left side and the same for the right side
      double error1 = distance+(motorPos[0] - RobotContainer.m_chasis.startPoints[0]);
      double drivePower1 = Math.min(Constants.DRIVE_AUTO_KP * error1, 1);
      double error2 = distance+(motorPos[1] - RobotContainer.m_chasis.startPoints[1]);
      double drivePower2 = Math.min(Constants.DRIVE_AUTO_KP * error1, 1);
      SmartDashboard.putNumber("drivePower", drivePower1);
      SmartDashboard.putNumber("max speed", maxSpeed);
      SmartDashboard.putNumber("error", error1);
      if (Math.abs(drivePower1) > maxSpeed) {
        drivePower1 = Math.copySign(maxSpeed, drivePower1);
      }
      if (Math.abs(drivePower2) > maxSpeed) {
        drivePower2 = Math.copySign(maxSpeed, drivePower2);
      }
      RobotContainer.m_chasis.moveLeft(-drivePower1);
      RobotContainer.m_chasis.moveRight(-drivePower2);
      if((Math.abs(error1) < tickThreshold&&Math.abs(error2) < tickThreshold)||(System.currentTimeMillis()-startTime)>maxTime) {
        isFinished = true;
      }
      maxSpeed +=0.01;
      SmartDashboard.putNumber("Degree Error", tickThreshold);
      SmartDashboard.putBoolean("Finish", isFinished);
      if(maxSpeed>motorSpeed) {
        maxSpeed = motorSpeed;
      }
    }
  }
  public double inchesToTicks(double inches) {
    return inches * (21504/(6*Math.PI));
  }
  public double angleToTicks(double angle) {
    return inchesToTicks(angle*(22 * Math.PI)/360);

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
    return isFinished;
  }
}
