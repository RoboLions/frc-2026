// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climb  {
  /** Creates a new Climb. */
  public static TalonFX Climber = new TalonFX(0);
// Here we are setting climb up at a given speed. 
  public static void RunClimber () {
    Climber.set( 0);
  }
//Here we are setting climb down at the given speed which is currently 0. 
  public static void LowerClimber () {
    Climber.set( 0);
  }

  public static void init(){
    TalonFXConfiguration climbMotorConfiguration = new TalonFXConfiguration();
    climbMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climbMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climbMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    climbMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    climbMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    climbMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    Climber.getConfigurator().apply(climbMotorConfiguration);

  }
}
