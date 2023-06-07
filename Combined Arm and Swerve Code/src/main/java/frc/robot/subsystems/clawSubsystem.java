// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class clawSubsystem extends SubsystemBase {
  DoubleSolenoid Solenoidclaw = new DoubleSolenoid(Constants.REVPH_ID, PneumaticsModuleType.CTREPCM, Constants.CLAW_SOLENOID_ID1, Constants.CLAW_SOLENOID_ID2);
  //DoubleSolenoid Solenoidclaw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CLAW_SOLENOID_ID1, Constants.CLAW_SOLENOID_ID2);
 
  public clawSubsystem() {
    //Solenoidclaw.set(Value.kReverse);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
  public void ClawPneuOut(){
    Solenoidclaw.set(Value.kForward);
    
  }
  public void ClawPneuIn(){
    Solenoidclaw.set(Value.kReverse);
  }
}
