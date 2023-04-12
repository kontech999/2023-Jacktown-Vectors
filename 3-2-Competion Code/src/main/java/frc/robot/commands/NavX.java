// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class NavX extends CommandBase {
  private double error;
  private double currentAngle;
  private double drivePower;
  private boolean start;
  private double pAngle;
  private double offset;
  private boolean reverseBalance;

  /** Creates a new NavX. */
  public NavX(boolean reverseBalance) {
    addRequirements(RobotContainer.m_chasis);
    this.reverseBalance = reverseBalance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = false;
    pAngle = 0;
    offset = Robot.m_robotContainer.getPitch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (start) {
      // Gets the angle of the robot
      this.currentAngle = Robot.m_robotContainer.getPitch() - offset;
      // this.currentAngle = this.currentAngle*0.7+pAngle * 0.3;
      // Do this code if the value ever goes past 180

      // if (Robot.m_robotContainer.getYaw() > 0) {
      // this.currentAngle = Robot.m_robotContainer.getYaw();
      // }else {
      // this.currentAngle = Robot.m_robotContainer.getYaw()+360;
      // }

      // Tells us how off we are from where we wanted to be
      error = Constants.BEAM_BALANCED_GOAL_DEGREES - currentAngle;
      if (reverseBalance) {
        error *= -1;
      }
      // double derative = pAngle-this.currentAngle;
      // This calculates how much power we give the motor based on the error
      drivePower = Math.min(Constants.BEAM_BALANACED_DRIVE_KP * error, 1);

      // Our robot needed an extra push to drive up in reverse, probably due to weight
      // imbalances

      // I comment out this line of code cause not sure if we will need this or not

      // if (drivePower < 0) {
      // drivePower *= Constants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
      // }

      // Limit the max power

      if (Math.abs(drivePower) > 0.06) {
        drivePower = Math.copySign(0.13, drivePower);
      }
      pAngle = this.currentAngle;
      // This uses the power variable to move it
      if (!reverseBalance) {
        RobotContainer.m_chasis.moveLeft(-drivePower);
        RobotContainer.m_chasis.moveRight(-drivePower);
      } else {
        RobotContainer.m_chasis.moveLeft(drivePower);
        RobotContainer.m_chasis.moveRight(drivePower);
      }
      SmartDashboard.putNumber("Angle", currentAngle);
      SmartDashboard.putNumber("Error", error);
      // SmartDashboard.putNumber("Derative", derative);
      SmartDashboard.putNumber("Drive Power", drivePower);
    } else {
      SmartDashboard.putNumber("Angle", Robot.m_robotContainer.getPitch() - offset);
      SmartDashboard.putNumber("Drive Power", drivePower);
      if (!reverseBalance) {
        RobotContainer.m_chasis.moveLeft(-0.25);
        RobotContainer.m_chasis.moveRight(-0.25);
      } else {
        RobotContainer.m_chasis.moveLeft(0.30);
        RobotContainer.m_chasis.moveRight(0.30);
      }
      this.currentAngle = Robot.m_robotContainer.getPitch() - offset;
      if (Math.abs(this.currentAngle - offset) > 9) {
        start = true;
      }

    }
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
    return false;
    // return Math.abs(error) < Constants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES;
    // End the command when we are within the specified threshold of being 'flat'
    // (gyroscope pitch of 0 degrees)
  }
}
