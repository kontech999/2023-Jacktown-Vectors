// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveOutLeft2Piece extends SequentialCommandGroup {
  /** Creates a new DriveOut2Piece. */
  public DriveOutLeft2Piece() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ClawHomePosition(0.67, 1500),
        new SlideAuto(13000, 1500),
        new ClawIn(),
        new WaitCommand(0.250),
        // new ParrelGroupMethod(
        new SlideAuto(300, 1500),
        new EncoderMoveForward(-130, 0.55, 4000),
        new EncoderRotate(-162, 0.7, 1500),
        new ClawHomePosition(0.80, 2000),
        new SlideAuto(12600, 2000),
        // ),
        new ClawOut(),
        new ClawHomePosition(0.67, 1500),
        new SlideAuto(300, 1500),
        new EncoderRotate(148, 0.7, 1500),
        new EncoderMoveForward(130, 0.55, 4000),
        new ClawHomePosition(0.67, 1500),
        new SlideAuto(13000, 1500),
        new ClawIn(),
        new StopAutoCommand());
  }
}
