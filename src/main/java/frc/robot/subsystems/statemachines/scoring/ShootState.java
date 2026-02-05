// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachines.scoring;

import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.Turret;

/** Add your docs here. */
public class ShootState extends State {

  @Override
  public void build() {
    addTransition(
        new Transition(
            () -> {
              return RobotMap.driverController.getBButton();
            },
            ScoringStateMachine.idleState));
  }

  @Override
  public void init(State prevState) {}

  @Override
  public void execute() {
    Turret.turretTrackHub();
  }
  
  @Override
  public void exit(State nextState) {

  }
}