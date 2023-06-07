// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleAutoV2 extends SequentialCommandGroup {
  /** Creates a new MiddleAutoV2. */
  public MiddleAutoV2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new ClawAuton(0.2, 500),
        new ClawHomePosition(0.67, 800),
        new SlideAuto(12310, 1000),
        new ClawIn(),
        new WaitCommand(0.250),
        new SlideAuto(300, 800),
        new ParrelGroupMethod(
            new ClawHomePosition(0.87, 1200),
            new EncoderMoveForward(-14, 0.55, 1000)),
        new EncoderRotate(180, .5, 2000),
        new NavX(false));
  }
}
