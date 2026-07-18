// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachines.scoring;

import frc.lib.statemachine.StateMachine;

/** Add your docs here. */
public class ScoringStateMachine extends StateMachine {

  public static IdleState idleState = new IdleState();
  public static IntakeState intakeState = new IntakeState();
  public static OuttakeState outtakeState = new OuttakeState();
  public static CycleState cycleState = new CycleState();

  public ScoringStateMachine() {
    intakeState.build();
    idleState.build();
    cycleState.build();
    outtakeState.build();
  }
}
