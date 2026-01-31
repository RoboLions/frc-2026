// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachines.scoring;

import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class PassAimState extends State {

  @Override
  public void build() {

  

    addTransition(
        new Transition(
            () -> {
              return RobotMap.driverController.getBButton();
            },
            ScoringStateMachine.idleState));
    addTransition(
        new Transition(
            () -> {
              return RobotMap.driverController.getRightTriggerAxis() > 0.25;
            },
            ScoringStateMachine.shootState));
  }

  @Override
  public void init(State prevState) {

  }

  @Override
  public void execute() {

  }

  @Override
  public void exit(State nextState) {

  }
}