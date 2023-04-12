// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class clawSubsystem extends SubsystemBase {
  CANSparkMax claw = new CANSparkMax(Constants.CLAW_MOTOR_ID, MotorType.kBrushless );
  DoubleSolenoid Solenoidclaw = new DoubleSolenoid(Constants.REVPH_ID, PneumaticsModuleType.CTREPCM, Constants.CLAW_SOLENOID_ID1, Constants.CLAW_SOLENOID_ID2);
  //DoubleSolenoid Solenoidclaw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CLAW_SOLENOID_ID1, Constants.CLAW_SOLENOID_ID2);
  /** Creates a new limelight. */
  public clawSubsystem() {
    Solenoidclaw.set(Value.kReverse);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
  public void clawmotor(double speed){
    claw.set(speed);
  }
  public void ClawPneuOut(){
    Solenoidclaw.set(Value.kForward);
    
  }
  public void ClawPneuIn(){
    Solenoidclaw.set(Value.kReverse);
  }
}
