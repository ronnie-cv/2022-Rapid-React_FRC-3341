// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class InfraredSensor extends SubsystemBase { 
  /** Creates a new InfraredSensor. */
  DigitalInput InfraredInput;

  public InfraredSensor() {
    InfraredInput = new DigitalInput(0);
  }

  public boolean get(){
    return !(InfraredInput.get());
    
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Ball Intaken", InfraredInput.get());
    // This method will be called once per scheduler run
  }
}
