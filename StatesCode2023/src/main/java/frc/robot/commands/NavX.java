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
  private boolean reverseBalance;
  private boolean PID;
  private double maxGetUpSpeed = 0;

  /** Creates a new NavX. */
  public NavX(boolean reverseBalance) {
    addRequirements(RobotContainer.m_chasis);
    this.reverseBalance = reverseBalance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_chasis.SetBrakeMode();
    start = false;
    pAngle = 0;
    PID = false;
    maxGetUpSpeed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (start) {
      // Gets the angle of the robot
      this.currentAngle = Robot.m_robotContainer.getPitch() - Constants.NAVX_OFFSET;
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
      
      // This calculates how much power we give the motor based on the error
      // drivePower = Math.min(Constants.BEAM_BALANACED_DRIVE_KP * error, 1);
      SmartDashboard.putNumber("Angle LOOK AT THIS", error);
      // Limit the max power
      SmartDashboard.putNumber("Drive Power", drivePower);
      // if (error > Constants.ANGLE_WHEN_PID_ENABLES) {
      /* 
        if (error > 7.5&&!PID) {
        drivePower = Math.copySign(drivePower * Constants.NAVX_SCALAR_VALUE, drivePower);
      }else {
        PID = true;
        //drivePower = 0;
      }*/
      if (error < 5) {
        drivePower = Math.copySign(error * Constants.BACK_NAVX_SCALAR_VALUE, error);
      }
      // else if(error>14){
      //   drivePower = Constants.CENTER_NAVX_SCALAR_VALUE;
      // }
      else{
        drivePower = Math.copySign(error * Constants.FORWARD_NAVX_SCALAR_VALUE, error);
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
    } else {
      SmartDashboard.putNumber("Angle", Robot.m_robotContainer.getPitch());
      SmartDashboard.putNumber("Drive Power", drivePower);
      if (!reverseBalance) {
        maxGetUpSpeed-=0.005;
        if (maxGetUpSpeed<-0.3) {
          maxGetUpSpeed = -0.3;
        }
        RobotContainer.m_chasis.moveLeft(maxGetUpSpeed);
        RobotContainer.m_chasis.moveRight(maxGetUpSpeed);
        // RobotContainer.m_chasis.moveLeft(-0.3);
        // RobotContainer.m_chasis.moveRight(-0.3);
      } else {
        RobotContainer.m_chasis.moveLeft(0.45);
        RobotContainer.m_chasis.moveRight(0.45);
      }
      this.currentAngle = Robot.m_robotContainer.getPitch() - Constants.NAVX_OFFSET;
      if (Math.abs(this.currentAngle) > Constants.ANGLE_WHEN_ENABLES) {
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
