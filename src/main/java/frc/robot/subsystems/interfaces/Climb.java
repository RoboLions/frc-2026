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

import frc.robot.Constants;

public class Climb  {
  public static TalonFX Climber = new TalonFX(Constants.CAN_IDS.CLIMB_MOTOR, "CANexternal");

  public static void init(){
    TalonFXConfiguration climbMotorConfiguration = new TalonFXConfiguration();

    climbMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climbMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climbMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 105;
    climbMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    climbMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    climbMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Climber.getConfigurator().apply(climbMotorConfiguration);
    Climber.setPosition(0.0);
  }

  public static void RunClimber () {
    Climber.set(0);
  }

  public static void LowerClimber () {
    Climber.set(0);
  }

}
