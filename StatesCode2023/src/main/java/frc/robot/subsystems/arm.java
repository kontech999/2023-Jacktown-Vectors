// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.commands.ArmCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class arm extends SubsystemBase {
  /** Creates a new arm. */
  private boolean clawMotorReverse = false;
  private CANSparkMax armMotor1;
  private CANSparkMax armMotor2;
  private CANSparkMax sled;
  private CANSparkMax bcMotor2;
  // This is the correct one
  private DutyCycleEncoder abEncoder;
  private Encoder bcEncoder;

  private DigitalInput limitSwitchExtend, limitSwitchRetract;

  // private CANEncoder abEncoder;
  // private CANEncoder bcEncoder;
  public boolean manualMode = true;
  public boolean auton = false;

  public arm() {
    // These declare the motors from the arm
    armMotor1 = new CANSparkMax(Constants.ARM_MOTOR_1, MotorType.kBrushless);
    armMotor2 = new CANSparkMax(Constants.ARM_MOTOR_2, MotorType.kBrushless);
    sled = new CANSparkMax(Constants.SLED_MOTOR_ID, MotorType.kBrushless);

    // This is the correct one
    abEncoder = new DutyCycleEncoder(Constants.ARM_ENCODER);
    bcEncoder = new Encoder(Constants.SLED_ENCODER_ID1, Constants.SLED_ENCODER_ID2);

    limitSwitchExtend = new DigitalInput(Constants.EXTEND_LIMITSWITCH_ID);
    limitSwitchRetract = new DigitalInput(Constants.RETRACT_LIMITSWITCH_ID);
  }

  public double getABEncoder() {
    // This is the correct format
    return (abEncoder.getAbsolutePosition() + Constants.ABOFFSET);
  }

  public double getBCEncoder() {
    // This is the correct format
    return bcEncoder.get();
    // return bcEncoder.getPosition();
  }

  public void setSledEncoderZero() {
    bcEncoder.reset();
  }

  public void moveArmMotors(double speed) {
    armMotor1.set(speed);
    armMotor2.set(-speed);
  }

  public void moveSledMotors(double speed) {
    sled.set(speed);
  }

  public boolean getExtendState() {
    // Rember true is when it not flipped
    return limitSwitchExtend.get();
  }

  public boolean getRetractState() {
    // Rember true is when it not flipped
    return limitSwitchRetract.get();
  }

  public double setABTicks(double ticks, double maxSpeed) {
    double currentTicks = getABEncoder();
    double error = ticks - currentTicks;
    double drivePower = Math.min(Constants.AB_MOTOR_DRIVE_KP * error, 1);
    if (Math.abs(drivePower) > maxSpeed) {
      drivePower = Math.copySign(maxSpeed, drivePower);
    }
    // abMoveMotors(-drivePower);
    return -drivePower;
  }

  public double setBCTicks(double ticks, double maxSpeed) {
    double currentTicks = getBCEncoder();
    double error = ticks - currentTicks;
    double drivePower = Math.min(Constants.SLED_MOTOR_DRIVE_KP * error, 1);
    if (Math.abs(drivePower) > maxSpeed) {
      drivePower = Math.copySign(maxSpeed, drivePower);
    }
    // bcMoveMotors(-drivePower);
    return drivePower;
  }

  public void changeManualMode() {
    manualMode = !manualMode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new ArmCommand());
  }
}
