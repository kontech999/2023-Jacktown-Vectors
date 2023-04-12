// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.impl.FailingSerializer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftAuto extends SequentialCommandGroup {
  /** Creates a new LeftAuto. */
  public LeftAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ClawAuton(0.2, 500),
        new EncoderMoveForward(-115, 0.55, 2700),
        new EncoderRotate(-90, 0.3, 1200),
        new EncoderMoveForward(40, 0.6, 1000),
        new EncoderRotate(90, 0.3, 1500),
        new NavX(false));
  }
}
