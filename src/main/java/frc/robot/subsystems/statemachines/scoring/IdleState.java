// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachines.scoring;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.LED;
import frc.robot.subsystems.interfaces.Shooter;

/** Add your docs here. */
public class IdleState extends State {

  @Override
  public void build() {
    addTransition(
            new Transition(
                () -> {
                return RobotMap.driverController.getBButtonPressed();
                },
                ScoringStateMachine.idleState));
    addTransition(
        new Transition(
            () -> {
              return RobotMap.driverController.getLeftTriggerAxis() > 0.25;
            },
              ScoringStateMachine.cycleState));
    addTransition(
            new Transition(
                () -> {
                return RobotMap.driverController.getRightBumper();
                },
                ScoringStateMachine.intakeState));
    addTransition(
        new Transition(
            () -> {
            return RobotMap.driverController.getLeftBumper();
            },
            ScoringStateMachine.outtakeState));
    addTransition(
        new Transition(
            () -> {
            return RobotMap.manipulatorController.getAButton();
            },
            ScoringStateMachine.outtakeState));
  }

  @Override
  public void init(State prevState) {
    Intake.allRollersStop();
    Shooter.idlerShooter();
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {LED.setSolidRed();} else {LED.setSolidRed();};
  }

  @Override
  public void execute() {
    if (RobotMap.driverController.getYButtonPressed() || RobotMap.manipulatorController.getLeftBumperButtonPressed()) {
      Intake.intakeUp();
      Intake.intakeSlow();
    }

    if (RobotMap.driverController.getYButtonReleased() || RobotMap.manipulatorController.getLeftBumperButtonReleased()) {
      Intake.stopIntake();
    }

    if (RobotMap.manipulatorController.getRightBumperButtonPressed()) {
      Intake.intakeDown();
    }
  }

  @Override
  public void exit(State nextState) {
        LED.turnOff();

  }
}