// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachines.scoring;

import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.Shooter;

/** Add your docs here. */
public class IdleState extends State {

  @Override
  public void build() {
    addTransition(
        new Transition(
            () -> {
              return RobotMap.driverController.getRightTriggerAxis() > 0.25;
            },
            ScoringStateMachine.shootState));
    addTransition(
            new Transition(
                () -> {
                return RobotMap.driverController.getRightBumper();
                },
                ScoringStateMachine.intakeState));
    addTransition(
        new Transition(
            () -> {
              return RobotMap.manipulatorController.getXButton();
            },
            ScoringStateMachine.presetState));
    addTransition(
        new Transition(
            () -> {
            return RobotMap.driverController.getLeftBumper();
            },
            ScoringStateMachine.outtakeState));
    addTransition(
        new Transition(
            () -> {
            return RobotMap.driverController.getLeftTriggerAxis() > 0.25;
            },
            ScoringStateMachine.passAimState));
    addTransition(
        new Transition(
            () -> {
            return RobotMap.driverController.getYButtonPressed();
            },
            ScoringStateMachine.testState));
  }

  @Override
  public void init(State prevState) {
    Intake.set(0);
    Intake.setFeed(0);
    Intake.setIndex(0);
    Shooter.stopAll();

    // simulation code
    Intake.simulateIntakeUp();
  }

  @Override
  public void execute() {
    if (RobotMap.driverController.getYButton()) {
      Intake.intakeUp();
    }
  }

  @Override
  public void exit(State nextState) {

  }
}