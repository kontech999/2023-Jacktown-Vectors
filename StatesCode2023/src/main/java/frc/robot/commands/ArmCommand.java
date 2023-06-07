// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  public double armEncoderPosition = 0.77;
  public double BCEncoderPosition = 0.55;
  public double BCMovePosition;
  public boolean debounce1;
  public boolean debounce2;
  public boolean debounce3;
  public double timerStart;
  private boolean start = true;
  boolean trigger = false;

  public ArmCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_arm.manualMode = true;
    // These debounces are for the triggers
    debounce1 = true;
    debounce2 = true;
    debounce3 = true;
    // This code makes sure that the arm never moves on initalize
    armEncoderPosition = RobotContainer.m_arm.getABEncoder();
    BCEncoderPosition = RobotContainer.m_arm.getBCEncoder();
    BCMovePosition = 0.40;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Limit Switch Extend", RobotContainer.m_arm.getExtendState());
    SmartDashboard.putBoolean("Limit Switch Retract", RobotContainer.m_arm.getRetractState());
    SmartDashboard.putNumber("arm encoder", RobotContainer.m_arm.getABEncoder());
    SmartDashboard.putNumber("bc encoder", RobotContainer.m_arm.getBCEncoder());
    // This set restrict the encoder value
    if (0.05 < RobotContainer.m_arm.getABEncoder() && RobotContainer.m_arm.getABEncoder() < 0.95&&!RobotContainer.m_arm.auton
    // && 0.05 < RobotContainer.m_arm.getBCEncoder() &&
    // RobotContainer.m_arm.getBCEncoder() < 0.95
    ) {
      double leftjoystick = Robot.m_robotContainer.GetXboxStickLeft();
      double rightjoystick = Robot.m_robotContainer.GetXboxStickRight();
      SmartDashboard.putNumber("arm encoder target", armEncoderPosition);
      SmartDashboard.putNumber("bc encoder target", BCEncoderPosition);
      SmartDashboard.putBoolean("Maunal Mode", RobotContainer.m_arm.manualMode);
      if (!RobotContainer.m_arm.getRetractState()) {
        RobotContainer.m_arm.setSledEncoderZero();
      }
      // intake
      if ((Robot.m_robotContainer.getdpad() || Robot.m_robotContainer.getXboxButton10()
          || Robot.m_robotContainer.getXboxButton9()) && debounce3) {
        //RobotContainer.m_arm.setClawBoolean(!RobotContainer.m_arm.getClawBoolean());
        debounce3 = false;
      }
      if ((!Robot.m_robotContainer.getdpad() && !Robot.m_robotContainer.getXboxButton10()
          && !Robot.m_robotContainer.getXboxButton9()) && !debounce3) {
        debounce3 = true;
      }
      // arm movement
      if (RobotContainer.m_arm.manualMode) {
        start = true;
        if (Math.abs(leftjoystick) > 0.05) {
          armEncoderPosition += leftjoystick * 0.001;
        } else {
          RobotContainer.m_arm.moveArmMotors(0);
        }
        if (Math.abs(rightjoystick) > 0.05) {
          // BCEncoderPosition += rightjoystick * 0.001;
          if (rightjoystick > 0 && RobotContainer.m_arm.getRetractState()) {
            RobotContainer.m_arm.moveSledMotors(rightjoystick * 0.25);
          } else if (rightjoystick < 0 && RobotContainer.m_arm.getExtendState()) {
            RobotContainer.m_arm.moveSledMotors(rightjoystick * 0.25);
          } else {
            RobotContainer.m_arm.moveSledMotors(0);
          }
        } else {
          RobotContainer.m_arm.moveSledMotors(0);
        }
        // This is just to make sure that it never hit the limit switch by accident
        // if(RobotContainer.m_arm.getRetractState()) {
        //   SmartDashboard.putNumber("Max Sled Number", RobotContainer.m_arm.getABEncoder()*-165643+147307);
        //   if(RobotContainer.m_arm.getABEncoder()*-164100+142921<Math.abs(RobotContainer.m_arm.getBCEncoder())){
        //     RobotContainer.m_arm.moveSledMotors(RobotContainer.m_arm.setBCTicks(RobotContainer.m_arm.getABEncoder()*-165643+147307, 0.6));
        //   }
        // }
        if (armEncoderPosition > Constants.MAX_ARM_ENCODER) {
          // This stop prevent the AB joint so it doesn't go over the height limit
          armEncoderPosition = Constants.MAX_ARM_ENCODER - 0.005;
        }
        if (armEncoderPosition < Constants.MIN_ARM_ENCODER) {
          // This stop prevent the AB joint so it doesn't go over the height limit
          armEncoderPosition = Constants.MIN_ARM_ENCODER + 0.005;
        }
        double power1 = RobotContainer.m_arm.setABTicks(armEncoderPosition, 0.6);
        //double power2 = RobotContainer.m_arm.setBCTicks(8000, 0.6);
        SmartDashboard.putNumber("ArmMotorPower", power1);
        RobotContainer.m_arm.moveArmMotors(-power1);
        BCMovePosition = BCEncoderPosition;
        // RobotContainer.m_arm.bcMoveMotors(power2);
        // RobotContainer.m_arm.setBCTicks(BCEncoderPosition, 0.15);
      } else { // auto movement
        if (Robot.m_robotContainer.isArmButtonPressed() != Math.PI
            && Robot.m_robotContainer.isArmButtonPressedBC() != Math.PI) {
          armEncoderPosition = Robot.m_robotContainer.isArmButtonPressed();
          // BCEncoderPosition = Robot.m_robotContainer.isArmButtonPressedBC();
          BCMovePosition = Robot.m_robotContainer.isArmButtonPressedBC();
          timerStart = 0;
          if ((armEncoderPosition == 0.87 && BCMovePosition == 300)
              || (armEncoderPosition == 0.80 && BCMovePosition == 1680)) {
            timerStart = System.currentTimeMillis();
          }
        }
        if (Robot.m_robotContainer.getJoyStickTriggerL() > 0.3 && debounce1) {
          armEncoderPosition -= 0.02;
          trigger = true;
          if (armEncoderPosition < Constants.MIN_ARM_ENCODER) {
            // This prevents the AB joint so it doesn't go over the height limit
            armEncoderPosition = Constants.MIN_ARM_ENCODER + 0.005;
          }
          debounce1 = false;
        }
        if (Robot.m_robotContainer.getJoyStickTriggerL() < 0.3) {
          debounce1 = true;
        }
        if (Robot.m_robotContainer.getJoyStickTriggerR() > 0.3 && debounce2) {
          armEncoderPosition += 0.01;
          trigger = true;
          if (armEncoderPosition > Constants.MAX_ARM_ENCODER) {
            // This prevents the AB joint so it doesn't go over the height limit
            armEncoderPosition = Constants.MAX_ARM_ENCODER - 0.005;
          }
          // This stop the trigger from having the arm go above the height limit
          debounce2 = false;
        }
        if (Robot.m_robotContainer.getJoyStickTriggerR() < 0.3) {
          debounce2 = true;
        }
        if (armEncoderPosition > Constants.MAX_ARM_ENCODER) {
          // This prevents the AB joint so it doesn't go over the height limit
          armEncoderPosition = Constants.MAX_ARM_ENCODER - 0.005;
        }
        if (armEncoderPosition < Constants.MIN_ARM_ENCODER) {
          // This prevents the AB joint so it doesn't go over the height limit
          armEncoderPosition = Constants.MIN_ARM_ENCODER + 0.005;
        }
        double power1 = RobotContainer.m_arm.setABTicks(armEncoderPosition, 0.6);
        if ((Math.abs(power1) > 0.05&&!trigger)||(Math.abs(power1) > 0.2&&trigger)) {
          trigger = false;
          BCEncoderPosition = 300;
          if (power1 < 0) {
            RobotContainer.m_arm.moveArmMotors(-power1);
          }
        } else if (trigger) {
          BCEncoderPosition = BCMovePosition;
        } else if (RobotContainer.m_arm.getABEncoder() < 0.85) {
          BCEncoderPosition = BCMovePosition;
        } else if (RobotContainer.m_arm.getABEncoder() > 0.85) {
          BCEncoderPosition = 300;
        }
        SmartDashboard.putNumber("Power AB", power1);
        double power2 = RobotContainer.m_arm.setBCTicks(BCEncoderPosition, 0.6);
        SmartDashboard.putNumber("Sled Power", power2);
        if (power2 < 0 && RobotContainer.m_arm.getRetractState()) {
          RobotContainer.m_arm.moveSledMotors(-power2);
        } else if (power2 > 0 && RobotContainer.m_arm.getExtendState()) {
          RobotContainer.m_arm.moveSledMotors(-power2);
        } else {
          RobotContainer.m_arm.moveSledMotors(0);
        }
        SmartDashboard.putNumber("Time Elapsed for down Delay", System.currentTimeMillis() - timerStart);
        if (System.currentTimeMillis() - timerStart > 1000) {
          RobotContainer.m_arm.moveArmMotors(-power1);
        } else {
          power1 = RobotContainer.m_arm.setABTicks(RobotContainer.m_arm.getABEncoder(), 0.6);
          // This is reverse on purpose
          RobotContainer.m_arm.moveArmMotors(power1);
        }
      }
    } else if (!RobotContainer.m_arm.auton) {
      RobotContainer.m_arm.moveArmMotors(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
