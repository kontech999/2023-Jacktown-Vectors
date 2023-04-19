// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
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
  private DutyCycleEncoder frontLeftEncoder;
  public chasis() {
    frontLeftMotor = new CANSparkMax(Constants.FRONT_LEFT_ID, MotorType.kBrushless);
    frontLeftRotateMotor = new CANSparkMax(Constants.FRONT_LEFT_ROTATE_ID, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(Constants.FRONT_RIGHT_ID, MotorType.kBrushless);
    frontRightRotateMotor = new CANSparkMax(Constants.FRONT_RIGHT_ROTATE_ID, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(Constants.BACK_LEFT_ID, MotorType.kBrushless);
    backLeftRotateMotor = new CANSparkMax(Constants.BACK_LEFT_ROTATE_ID, MotorType.kBrushless);
    backRightMotor = new CANSparkMax(Constants.BACK_RIGHT_ID, MotorType.kBrushless);
    backRightRotateMotor = new CANSparkMax(Constants.BACK_RIGHT_ROTATE_ID, MotorType.kBrushless);
    frontLeftEncoder = new DutyCycleEncoder(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("frontLeftEncoderValue", frontLeftEncoder.getAbsolutePosition());
  }
}
