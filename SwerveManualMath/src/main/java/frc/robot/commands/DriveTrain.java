// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveTrain extends CommandBase {

  public double FWD, STR, ROT;
  public double robotAngle = 0;
  public double L = 26;
  public double W = 22;
  public double R = Math.sqrt(L * L + W * W);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    FWD = Robot.m_robotContainer.getFWD() * Math.cos(robotAngle)
        + Robot.m_robotContainer.getSTR() * Math.sin(robotAngle);
    STR = Robot.m_robotContainer.getSTR() * Math.cos(robotAngle)
        - Robot.m_robotContainer.getFWD() * Math.sin(robotAngle);
    ROT = Robot.m_robotContainer.getROT();

    double A = STR - ROT * (L / R);
    double B = STR + ROT * (L / R);
    double C = FWD - ROT * (W / R);
    double D = FWD + ROT * (W / R);

    double ws_FR = Math.sqrt(B * B + C * C);
    double ws_FL = Math.sqrt(B * B + D * D);
    double ws_BR = Math.sqrt(A * A + C * C);
    double ws_BL = Math.sqrt(A * A + D * D);

    double wa_FR = Math.atan2(B, C);
    double wa_FL = Math.atan2(B, D);
    double wa_BR = Math.atan2(A, C);
    double wa_BL = Math.atan2(A, D);

    double ws_MAX = Math.max(Math.max(ws_FR, ws_FL), Math.max(ws_BR, ws_BL));
    if (ws_MAX > 1.0) {
      ws_FR = ws_FR/ws_MAX;
      ws_FL = ws_FL/ws_MAX;
      ws_BR = ws_BR/ws_MAX;
      ws_BL = ws_BL/ws_MAX;
    }

    RobotContainer.m_chasis.updateFrontLeft(ws_FL, wa_FL);
    RobotContainer.m_chasis.updateFrontRight(ws_FR, wa_FR);
    RobotContainer.m_chasis.updateBackLeft(ws_BL, wa_BL);
    RobotContainer.m_chasis.updateBackRight(ws_BR, wa_BR);

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
