// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachines.scoring;

import frc.robot.lib.statemachine.StateMachine;

/** Add your docs here. */
public class ScoringStateMachine extends StateMachine{

    public static IdleState idleState = new IdleState();
    public static IntakeState intakeState = new IntakeState();
    public static OuttakeState outtakeState = new OuttakeState();
    public static HubAimState hubAimState = new HubAimState();
    public static PassAimState passAimState = new PassAimState();
    public static ShootState shootState = new ShootState();
    public static PresetState presetState = new PresetState();
    public static HubCycleState hubCycleState = new HubCycleState();
    public static PassCycleState passCycleState = new PassCycleState();

    

    public ScoringStateMachine() {
        intakeState.build();
        idleState.build();
        hubAimState.build();
        passAimState.build();
        shootState.build();
        presetState.build();
        hubCycleState.build();
        passCycleState.build();
    }
}
