// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachines.drivetrain;

import frc.robot.lib.statemachine.StateMachine;

/** Add your docs here. */
public class DrivetrainStateMachine extends StateMachine{

    public static TeleopState teleopState = new TeleopState();
    public static AlignState alignState = new AlignState();
    public static BrakeState brakeState = new BrakeState();
    public static ShootMoveState shootmoveState = new ShootMoveState();

    public DrivetrainStateMachine() {
        teleopState.build();
        alignState.build();
        brakeState.build();
        shootmoveState.build();
    }
}
