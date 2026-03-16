// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachines.scoring;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.Shooter;
import frc.robot.subsystems.interfaces.Turret;

/** Add your docs here. */
public class IdleState extends State {

  @Override
  public void build() {
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
  }

  @Override
  public void init(State prevState) {
    Shooter.idlerShooter();
    Intake.intakeMid();

    RobotMap.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
  }

  @Override
  public void execute() {
    Intake.allRollersStop();

    if (RobotMap.manipulatorController.getRightTriggerAxis() > 0.25) {
      Intake.FeedOut();
      Intake.IndexOut();
    }

    if (RobotMap.manipulatorController.getAButton()) {
      Intake.intakeUp();
      Turret.setAzimuthZero();
    }
  }

  @Override
  public void exit(State nextState) {

  }
}