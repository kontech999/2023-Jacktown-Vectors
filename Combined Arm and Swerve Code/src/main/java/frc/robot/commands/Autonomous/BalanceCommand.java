// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {

	private static DriveSubsystem driveSubsystem;

	private final ProfiledPIDController drivePIDController;
	private final ProfiledPIDController drivePIDControllerBackwards;
	private boolean isEngaged = false;
	double offset;
	boolean end;

	/** Creates a new BalanceCommand. */
	public BalanceCommand() {

		driveSubsystem = RobotContainer.driveSubsystem;

		drivePIDController = new ProfiledPIDController(
				AutoConstants.kBalanceCommandGains.kP,
				AutoConstants.kBalanceCommandGains.kI,
				AutoConstants.kBalanceCommandGains.kD,
				new TrapezoidProfile.Constraints(AutoConstants.kMaxBalancingVelocity,
						AutoConstants.kMaxBalancingAcceleration));
		drivePIDControllerBackwards = new ProfiledPIDController(
				0.007,
				AutoConstants.kBalanceCommandGains.kI,
				AutoConstants.kBalanceCommandGains.kD,
				new TrapezoidProfile.Constraints(AutoConstants.kMaxBalancingVelocity,
						AutoConstants.kMaxBalancingAcceleration));

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(driveSubsystem);
		offset = driveSubsystem.getRoll() + driveSubsystem.getPitch();
		isEngaged = false;
		end = false;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		driveSubsystem.setFieldCentric(true);
		offset = driveSubsystem.getRoll() + driveSubsystem.getPitch();
		isEngaged = false;
		end = false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		SmartDashboard.putString("I am trying to balance", " ");
		// Roll is associated with driving
		// Pitch is associated with strafing
		double driveOutput = !isEngaged ? -0.3 : 0;

		// checks to see if the charge station is close to balanced
		if (Math.abs(driveSubsystem.getRoll() + driveSubsystem.getPitch()
				- offset) > 3.5) {
			if (driveSubsystem.getRoll() + driveSubsystem.getPitch()
					- offset < AutoConstants.kBalnaceCommandDeadbandDeg) {
				// sets driveoutput to the output of the pid controller if the station is not
				// balanced
				driveOutput = -drivePIDController.calculate(driveSubsystem.getPitch(), 0);
				// driveOutput = -0.15;
				isEngaged = true;
			} else {
				driveOutput = -drivePIDControllerBackwards.calculate(driveSubsystem.getPitch(), 0);
				isEngaged = true;
				// end = true;
			}
		}
		SmartDashboard.putNumber("Balance Power", driveOutput);
		SmartDashboard.putNumber("Current Angle", driveSubsystem.getRoll() + driveSubsystem.getPitch() - offset);
		// add strafe output here to have the robot strafe while balancing
		driveSubsystem.drive(-driveOutput, 0, 0);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// turns the wheels when the command ends
		driveSubsystem.drive(0, 0, 0.001);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
