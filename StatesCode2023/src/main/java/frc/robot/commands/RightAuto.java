// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightAuto extends SequentialCommandGroup {
  /** Creates a new FirstAuto. */
  public RightAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ClawHomePosition(0.67, 800),
        new SlideAuto(12310, 800),
        new ClawIn(),
        new WaitCommand(0.250),
        new ParrelGroupMethod(
            new SequentialCommandGroup(new SlideAuto(300, 800), new ClawHomePosition(0.87, 1200)),
            new EncoderMoveForward(-110, 0.55, 2700)),
        new EncoderRotate(90, 0.7, 800),
        new EncoderMoveForward(30, 0.6, 800),
        new EncoderRotate(-90, 0.7, 800),
        new NavX(false));

    // addCommands(new EncoderMoveForward(168, 0.3, 1000), new StopAutoCommand());
  }
}
