// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.commands.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class chasis extends SubsystemBase {
  /** Creates a new chasis. */
  private CANSparkMax frontLeftMotor;
  private CANSparkMax frontLeftRotateMotor;
  private CANSparkMax frontRightMotor;
  private CANSparkMax frontRightRotateMotor;
  private CANSparkMax backLeftMotor;
  private CANSparkMax backLeftRotateMotor;
  private CANSparkMax backRightMotor;
  private CANSparkMax backRightRotateMotor;
  public AnalogInput frontLeftEncoder;
  public AnalogInput frontRightEncoder;
  public AnalogInput backLeftEncoder;
  public AnalogInput backRightEncoder;
  public SwerveModule frontLeft, frontRight, backLeft, backRight;

  public chasis() {
    frontLeftMotor = new CANSparkMax(Constants.FRONT_LEFT_ID, MotorType.kBrushless);
    frontLeftRotateMotor = new CANSparkMax(Constants.FRONT_LEFT_ROTATE_ID, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(Constants.FRONT_RIGHT_ID, MotorType.kBrushless);
    frontRightRotateMotor = new CANSparkMax(Constants.FRONT_RIGHT_ROTATE_ID, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(Constants.BACK_LEFT_ID, MotorType.kBrushless);
    backLeftRotateMotor = new CANSparkMax(Constants.BACK_LEFT_ROTATE_ID, MotorType.kBrushless);
    backRightMotor = new CANSparkMax(Constants.BACK_RIGHT_ID, MotorType.kBrushless);
    backRightRotateMotor = new CANSparkMax(Constants.BACK_RIGHT_ROTATE_ID, MotorType.kBrushless);

    frontLeftEncoder = new AnalogInput(0);
    frontRightEncoder = new AnalogInput(1);
    backLeftEncoder = new AnalogInput(3);
    backRightEncoder = new AnalogInput(2);

    frontLeft = new SwerveModule(frontLeftRotateMotor, frontLeftMotor, frontLeftEncoder);
    frontRight = new SwerveModule(frontRightRotateMotor, frontRightMotor, frontRightEncoder);
    backLeft = new SwerveModule(backLeftRotateMotor, backLeftMotor, backLeftEncoder);
    backRight = new SwerveModule(backRightRotateMotor, backRightMotor, backRightEncoder);

  }

  public void updateFrontLeft(double speed, double angle) {
    frontLeft.setSpeed(speed);
    frontLeft.setAngle(angle);
  }

  public void updateFrontRight(double speed, double angle) {
    frontRight.setSpeed(speed);
    frontRight.setAngle(angle);
  }

  public void updateBackLeft(double speed, double angle) {
    backLeft.setSpeed(speed);
    backLeft.setAngle(angle);
  }

  public void updateBackRight(double speed, double angle) {
    backRight.setSpeed(speed);
    backRight.setAngle(angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("frontLeftEncoderValue", frontLeftEncoder.getVoltage());
    SmartDashboard.putNumber("frontRightEncoderValue", frontRightEncoder.getVoltage());
    SmartDashboard.putNumber("backLeftEncoderValue", backLeftEncoder.getVoltage());
    SmartDashboard.putNumber("backRightEncoderValue", backRightEncoder.getVoltage());
    setDefaultCommand(new DriveTrain());
  }
}

class SwerveModule {
  CANSparkMax rotateMotor, powerMotor;
  PIDController pid;
  AnalogInput encoder;

  SwerveModule(CANSparkMax rotateMotor, CANSparkMax powerMotor, AnalogInput encoder) {
    this.powerMotor = powerMotor;
    this.rotateMotor = rotateMotor;
    this.encoder = encoder;
    pid = new PIDController(0.01, 0, 0);
  }

  public void setSpeed(double speed) {
    powerMotor.set(speed);
  }

  public void setAngle(double angle) {
    if (angle < 0) {
      angle += 5;
    }
    double error = Math.IEEEremainder(encoder.getVoltage() - angle, 5);
    if (error > 5 / 4) {
    }
    rotateMotor.set(error * 0.4);
  
  }
}
