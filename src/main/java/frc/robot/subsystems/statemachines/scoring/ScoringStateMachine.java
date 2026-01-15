// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachines.scoring;

/** Add your docs here. */
public class ScoringStateMachine {

    public static IdleState idleState = new IdleState();

    public ScoringStateMachine() {
        idleState.build();
    }
}
