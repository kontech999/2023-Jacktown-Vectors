// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.commands.ArmCommand;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class arm extends SubsystemBase {
  /** Creates a new arm. */
  private boolean clawMotorReverse = false;
  private CANSparkMax abMotor1;
  private CANSparkMax abMotor2;
  private CANSparkMax bcMotor1;
  private CANSparkMax bcMotor2;
  // This is the correct one
  private DutyCycleEncoder abEncoder;
  private DutyCycleEncoder bcEncoder;

  // private CANEncoder abEncoder;
  // private CANEncoder bcEncoder;
  public boolean manualMode = true;

  public arm() {
    abMotor1 = new CANSparkMax(Constants.AB_MOTOR_1, MotorType.kBrushless);
    abMotor2 = new CANSparkMax(Constants.AB_MOTOR_2, MotorType.kBrushless);
    bcMotor1 = new CANSparkMax(Constants.BC_MOTOR_1, MotorType.kBrushless);
    bcMotor2 = new CANSparkMax(Constants.BC_MOTOR_2, MotorType.kBrushless);

    // This is the correct one
    abEncoder = new DutyCycleEncoder(Constants.AB_ENCODER);
    bcEncoder = new DutyCycleEncoder(Constants.BC_ENCODER);
    // bcEncoder = new DutyCycleEncoder(1);

    // Get rid of this for bot actual
    // abEncoder = abMotor1.getEncoder();
    // bcEncoder = bcMotor1.getAb();
  }

  public double getABEncoder() {
    // This is the correct format
    return (abEncoder.getAbsolutePosition()+Constants.ABOFFSET);
    // return bcEncoder.getPosition();
  }

  public boolean isABEncoderConnected() {
    // This is the correct format
    // return abEncoder.isConnected();
    return true;
  }

  public double getBCEncoder() {
    // This is the correct format
    return bcEncoder.getAbsolutePosition();
    // return bcEncoder.getPosition();
  }

  public void abMoveMotors(double speed) {
    abMotor1.set(speed);
    abMotor2.set(-speed);
  }

  public void bcMoveMotors(double speed) {
    bcMotor1.set(speed);
    bcMotor2.set(-speed);
  }

  public double setABTicks(double ticks, double maxSpeed) {
    double currentTicks = getABEncoder();
    double error = ticks - currentTicks;
    double drivePower = Math.min(Constants.AB_MOTOR_DRIVE_KP * error, 1);
    if (Math.abs(drivePower) > maxSpeed) {
      drivePower = Math.copySign(maxSpeed, drivePower);
    }
    //abMoveMotors(-drivePower);
    return -drivePower;
  }

  public double setBCTicks(double ticks, double maxSpeed) {
    double currentTicks = getBCEncoder();
    double error = ticks - currentTicks;
    double drivePower = Math.min(Constants.BC_MOTOR_DRIVE_KP * error, 1);
    if (Math.abs(drivePower) > maxSpeed) {
      drivePower = Math.copySign(maxSpeed, drivePower);
    }
    //bcMoveMotors(-drivePower);
    return -drivePower;
  }

  public void changeManualMode() {
    manualMode = !manualMode;
  }

  public void setClawBoolean(boolean bool) {
    clawMotorReverse = bool;
  }

  public boolean getClawBoolean() {
    return clawMotorReverse;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new ArmCommand());
  }
}
