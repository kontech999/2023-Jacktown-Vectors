// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.TractorToolbox.TractorParts;

import java.io.File;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.AutoConstants.PathPLannerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class PathBuilder {

	private static DriveSubsystem driveSubsystem;
	private static SwerveAutoBuilder autoBuilder;

	private static HashMap<String, Command> pathMap = new HashMap<>();

	public PathBuilder() {
		driveSubsystem = RobotContainer.driveSubsystem;

		autoBuilder = new SwerveAutoBuilder(
				driveSubsystem::getPose,
				driveSubsystem::resetOdometry,
				DriveConstants.kDriveKinematics,
				new PIDConstants(
						PathPLannerConstants.kPPDriveGains.kP,
						PathPLannerConstants.kPPDriveGains.kI,
						PathPLannerConstants.kPPDriveGains.kD),
				new PIDConstants(
						PathPLannerConstants.kPPTurnGains.kP,
						PathPLannerConstants.kPPTurnGains.kI,
						PathPLannerConstants.kPPTurnGains.kD),
				driveSubsystem::setModuleStates,
				PathPLannerConstants.kPPEventMap,
				false,
				driveSubsystem);
	}

	public Command getPathCommand(String path) {
		return pathMap.get(path);
	}

	public void addPath(String pathName) {
		pathMap.put(
					pathName,
					autoBuilder.fullAuto(PathPlanner.loadPathGroup(
							pathName,
							new PathConstraints(
									PathPLannerConstants.kPPMaxVelocity,
									PathPLannerConstants.kPPMaxAcceleration))));
	}

	/** fills the path map with paths from the pathplanner directory */
	public void populatePathMap() {
		String[] pathNames = {"Square", "New Path", "PathPlanner Fun"};
		for (int i = 0; i < pathNames.length; i++) {
			String pathName = pathNames[i];
			pathMap.put(
					pathName,
					autoBuilder.fullAuto(PathPlanner.loadPathGroup(
							pathName,
							new PathConstraints(
									PathPLannerConstants.kPPMaxVelocity,
									PathPLannerConstants.kPPMaxAcceleration))));
		}
	}

}
