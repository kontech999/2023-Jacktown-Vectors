// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.xml.stream.events.StartDocument;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  //What the hell are you doing?
  public double ABEncoderPosition = 0.77;
  public double BCEncoderPosition = 0.55;
  public double BCMovePosition;
  public boolean debounce1;
  public boolean debounce2;
  public boolean debounce3;
  public double timerStart;
  private boolean start = true;
  private boolean autoStart = false;
  public ArmCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_arm.manualMode = true;
    debounce1 = true;
    debounce2 = true;
    debounce3 = true;
    ABEncoderPosition = RobotContainer.m_arm.getABEncoder();
    BCEncoderPosition = RobotContainer.m_arm.getBCEncoder();
    BCMovePosition = 0.40;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This set restrict the encoder value
    if (0.05 < RobotContainer.m_arm.getABEncoder() && RobotContainer.m_arm.getABEncoder() < 0.95
        && 0.05 < RobotContainer.m_arm.getBCEncoder() && RobotContainer.m_arm.getBCEncoder() < 0.95) {
      double leftjoystick = Robot.m_robotContainer.GetXboxStickLeft();
      double rightjoystick = Robot.m_robotContainer.GetXboxStickRight();
      SmartDashboard.putNumber("ab encoder", RobotContainer.m_arm.getABEncoder());
      SmartDashboard.putNumber("bc encoder", RobotContainer.m_arm.getBCEncoder());
      SmartDashboard.putNumber("ab encoder target", ABEncoderPosition);
      SmartDashboard.putNumber("bc encoder target", BCEncoderPosition);
      SmartDashboard.putBoolean("Arm Reverse", RobotContainer.m_arm.getClawBoolean());
      SmartDashboard.putBoolean("Maunal Mode", RobotContainer.m_arm.manualMode);
      SmartDashboard.putBoolean("Start", start);
      SmartDashboard.putNumber("StartTime", timerStart);
      // intake
      if ((Robot.m_robotContainer.getdpad() || Robot.m_robotContainer.getXboxButton10()
          || Robot.m_robotContainer.getXboxButton9()) && debounce3) {
        RobotContainer.m_arm.setClawBoolean(!RobotContainer.m_arm.getClawBoolean());
        debounce3 = false;
      }
      if ((!Robot.m_robotContainer.getdpad() && !Robot.m_robotContainer.getXboxButton10()
          && !Robot.m_robotContainer.getXboxButton9()) && !debounce3) {
        debounce3 = true;
      }
      // arm movement
      if (RobotContainer.m_arm.manualMode) {
        start = true;
        autoStart = false;
        if (Math.abs(leftjoystick) > 0.05) {
          ABEncoderPosition -= leftjoystick * 0.001;
        } else {
          RobotContainer.m_arm.abMoveMotors(0);
        }
        if (Math.abs(rightjoystick) > 0.05) {
          BCEncoderPosition += rightjoystick * 0.001;
        }
        // BCEncoderPosition = 0.702 * ABEncoderPosition;
        if (ABEncoderPosition > 0.86) {
          // This stop prevent the AB joint so it doesn't go over the height limit
          ABEncoderPosition = 0.85;
        }
        double power1 = RobotContainer.m_arm.setABTicks(ABEncoderPosition, 0.3);
        double power2 = RobotContainer.m_arm.setBCTicks(BCEncoderPosition, 0.3);
        RobotContainer.m_arm.abMoveMotors(power1);
        RobotContainer.m_arm.bcMoveMotors(power2);
        // RobotContainer.m_arm.setBCTicks(BCEncoderPosition, 0.15);
      } else { // auto movement
        if (Robot.m_robotContainer.isArmButtonPressedAB() != Math.PI
            && Robot.m_robotContainer.isArmButtonPressedBC() != Math.PI) {
          ABEncoderPosition = Robot.m_robotContainer.isArmButtonPressedAB();
          // BCEncoderPosition = Robot.m_robotContainer.isArmButtonPressedBC();
          BCMovePosition = Robot.m_robotContainer.isArmButtonPressedBC();
          timerStart = 0;
          if (ABEncoderPosition == 0.54 && BCEncoderPosition == 0.4&&!start) {
            timerStart = System.currentTimeMillis();
          }
          autoStart = true;
          // timerStart = System.currentTimeMillis();
        }
        if (!(ABEncoderPosition == 0.54) && !(BCEncoderPosition == 0.4) && start&&autoStart) {
          start = false;
        }
        if (Robot.m_robotContainer.getJoyStickTriggerL() > 0.3 && debounce1) {
          // ABEncoderPosition -= 0.01;
          BCMovePosition += 0.02;
          if (BCMovePosition < 0.44) {
            BCMovePosition = 0.45;
          }
          debounce1 = false;
        }
        if (Robot.m_robotContainer.getJoyStickTriggerL() < 0.3) {
          debounce1 = true;
        }
        if (Robot.m_robotContainer.getJoyStickTriggerR() > 0.3 && debounce2) {
          // ABEncoderPosition += 0.03;
          BCMovePosition -= 0.09;
          if (BCMovePosition < 0.44) {
            BCMovePosition = 0.45;
          }
          // This stop the trigger from having the arm go above the height limit
          if (ABEncoderPosition > 0.86) {
            ABEncoderPosition = 0.85;
          }
          debounce2 = false;
        }
        if (Robot.m_robotContainer.getJoyStickTriggerR() < 0.3) {
          debounce2 = true;
        }
        double power1 = RobotContainer.m_arm.setABTicks(ABEncoderPosition, 0.5);
        if ((Math.abs(power1) > 0.05 && 0.66 < RobotContainer.m_arm.getABEncoder()
            && RobotContainer.m_arm.getABEncoder() < 0.78)) {
          // BCEncoderPosition = 0.742 * ABEncoderPosition;
          BCEncoderPosition = 0.40;
          if (power1 < 0) {
            RobotContainer.m_arm.abMoveMotors(power1);
          }
        } else if (RobotContainer.m_arm.getABEncoder() > 0.66) {
          BCEncoderPosition = BCMovePosition;
        } else if (RobotContainer.m_arm.getABEncoder() < 0.66) {
          BCEncoderPosition = 0.40;
        }
        SmartDashboard.putNumber("Power AB", power1);
        double power2 = RobotContainer.m_arm.setBCTicks(BCEncoderPosition, 0.5);
        if(System.currentTimeMillis() - timerStart>1000) {
          RobotContainer.m_arm.abMoveMotors(power1);
        }else {
          RobotContainer.m_arm.abMoveMotors(0);
        }
        //RobotContainer.m_arm.abMoveMotors(power1);
        RobotContainer.m_arm.bcMoveMotors(power2 * 0.5);
      }
    } else {
      RobotContainer.m_arm.abMoveMotors(0);
      RobotContainer.m_arm.bcMoveMotors(0);
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
