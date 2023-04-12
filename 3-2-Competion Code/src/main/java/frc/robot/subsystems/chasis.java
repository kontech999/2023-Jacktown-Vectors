// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveTrain;

public class chasis extends SubsystemBase {
  /** Creates a new chasis. */
  private TalonFX frontLeftMotor;
  private TalonFX frontRightMotor;
  private TalonFX backLeftMotor;
  private TalonFX backRightMotor;
  public double[] startPoints = new double[4];
  public double[] motorPosition = new double[4];

  public chasis() {
    // Declares the four driving motors as a variable
    frontLeftMotor = new TalonFX(Constants.FRONT_LEFT_ID);
    frontRightMotor = new TalonFX(Constants.FRONT_RIGHT_ID);

    backLeftMotor = new TalonFX(Constants.BACK_LEFT_ID);
    backRightMotor = new TalonFX(Constants.BACK_RIGHT_ID);

    frontLeftMotor.setNeutralMode(NeutralMode.Coast);
    frontRightMotor.setNeutralMode(NeutralMode.Coast);
    backLeftMotor.setNeutralMode(NeutralMode.Coast);
    backRightMotor.setNeutralMode(NeutralMode.Coast);
    // Left motors are inverted so this corrects it
    frontLeftMotor.setInverted(true);
    backLeftMotor.setInverted(true);

  }

  public void initializeMotors() {

    initializeMotor(frontLeftMotor, 0);
    initializeMotor(frontRightMotor, 1);
    initializeMotor(backLeftMotor, 2);
    initializeMotor(backRightMotor, 3);

  }

  private void initializeMotor(TalonFX motor, int index) {
    motor.configFactoryDefault();
    motor.configNeutralDeadband(0.001);
    startPoints[index] = motor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    setDefaultCommand(new DriveTrain());
    // This method will be called once per scheduler run
  }

  // This moves the left motors
  public void moveLeft(double speed) {
    frontLeftMotor.set(ControlMode.PercentOutput, speed);
    backLeftMotor.set(ControlMode.PercentOutput, speed);

  }

  // This moves the right motors
  public void moveRight(double speed) {
    frontRightMotor.set(ControlMode.PercentOutput, speed);
    backRightMotor.set(ControlMode.PercentOutput, speed);
  }

  public double[] getMotorPosition() {
    motorPosition[0] = frontLeftMotor.getSelectedSensorPosition();
    motorPosition[1] = frontRightMotor.getSelectedSensorPosition();
    motorPosition[2] = backLeftMotor.getSelectedSensorPosition();
    motorPosition[3] = backRightMotor.getSelectedSensorPosition();
    return motorPosition;
  }

  public void updateStartPoints() {
    startPoints[0] = frontLeftMotor.getSelectedSensorPosition();
    startPoints[1] = frontRightMotor.getSelectedSensorPosition();
    startPoints[2] = backLeftMotor.getSelectedSensorPosition();
    startPoints[3] = backRightMotor.getSelectedSensorPosition();
  }

  // this was added last night but is not being called.
  // we need t oset up a push button to toggle between coast mode and
  // brake mode.
  public void SetCoastMode() {
    frontLeftMotor.setNeutralMode(NeutralMode.Coast);
    frontRightMotor.setNeutralMode(NeutralMode.Coast);
    backLeftMotor.setNeutralMode(NeutralMode.Coast);
    backRightMotor.setNeutralMode(NeutralMode.Coast);

  }

  public void SetBrakeMode() {
    frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    backLeftMotor.setNeutralMode(NeutralMode.Brake);
    backRightMotor.setNeutralMode(NeutralMode.Brake);
  }
}
