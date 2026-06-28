// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachines.scoring;

import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.LED;
import frc.robot.subsystems.interfaces.Shooter;

/** Add your docs here. */
public class IntakeState extends State {

  @Override
  public void build() {
    addTransition(
        new Transition(
            () -> {
              return RobotMap.driverController.getLeftBumperButton();
            },
            ScoringStateMachine.outtakeState));
    addTransition(
        new Transition(
            () -> {
              return RobotMap.driverController.getBButton();
            },
            ScoringStateMachine.idleState));
    addTransition(
        new Transition(
            () -> {
              return RobotMap.driverController.getLeftTriggerAxis() > 0.25;
            },
            ScoringStateMachine.cycleState));
  }
  @Override
  public void init(State prevState) {
    Intake.intake();
    Intake.intakeDown();
    Shooter.idlerShooter();
    LED.setFlashYellow();
  }

  @Override
  public void execute() {
    if (RobotMap.manipulatorController.getBButtonPressed()) {
      Intake.stopIntake();
    } else if (RobotMap.manipulatorController.getBButtonReleased()) {
      Intake.intake();
    }
  } 

  @Override
  public void exit(State nextState) {}
}