package frc.robot.subsystems.statemachines.scoring;

import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.Shooter;
import frc.robot.subsystems.interfaces.Turret;
import frc.robot.subsystems.interfaces.Turret.SimulationObjects;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

public class TestState extends State {

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
    Intake.FeedIn();
    Shooter.setShootSpeed(SimulationObjects.totalShotVelocity);

    if (RobotMap.driverController.getLeftBumperButtonPressed()) {
      Intake.FeedOut();
    }

    if (RobotMap.driverController.getRightTriggerAxis() > 0.25) {
      Intake.allRollersIn();
    }
  }

  @Override
  public void exit(State nextState) {

  }
}
