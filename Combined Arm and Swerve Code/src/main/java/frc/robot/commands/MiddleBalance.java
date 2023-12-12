// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Autonomous.BalanceCommand;
import frc.robot.commands.Autonomous.TimedDriveCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleBalance extends SequentialCommandGroup {
	/** Creates a new MiddleBalance. */
	public MiddleBalance() {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(new ClawHomePosition(0.66, 800),
				new SlideAuto(12310, 800),
				new ClawOut(),
				new WaitCommand(0.250),
				new SlideAuto(300, 800), 
				new ParallelGroupMethod(
					new ClawHomePosition(0.87, 800), 
					new TimedDriveCommand(-.4, 0, 0, 3800)),
				new WaitCommand(0.5)
				,new BalanceCommand()
				);
	}
}
