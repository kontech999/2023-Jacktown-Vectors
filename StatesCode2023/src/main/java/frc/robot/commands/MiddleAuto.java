// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleAuto extends SequentialCommandGroup {
  /** Creates a new MiddleAuto. */
  public MiddleAuto() {
    addCommands(
        new ClawHomePosition(0.67, 800),
        new SlideAuto(12310, 1000),
        new ClawIn(),
        new WaitCommand(0.250),
        new ParallelCommandGroup(
            new SlideAuto(300, 800),
            new AngledAutonMove(-3, false)),
        new ParallelCommandGroup(
            new MoveTimeAuto(0.3, 900),
            new ClawHomePosition(0.87, 800)

        ),
        new NavX(false));

  }
}
