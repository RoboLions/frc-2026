// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachines.scoring;

import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.Shooter;
import frc.robot.subsystems.interfaces.Turret;
import frc.robot.subsystems.swerve.Swerve;

/** Add your docs here. */
public class IdleState extends State {

  @Override
  public void build() {
    addTransition(
        new Transition(
            () -> {
              return RobotMap.driverController.getRightTriggerAxis() > 0.25;
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
    Intake.allRollersStop();
    Shooter.idlerShooter();
  }

  @Override
  public void execute() {
    if (Swerve.getPose().getX() >= 4.75 && Swerve.getPose().getX() <= 11.75) {
      Turret.turretTrackPassPoseAzimuth();
    } else {
      Turret.turretTrackHubAzimuth();
    }
    
    if (RobotMap.driverController.getYButton()) {
      Intake.intakeUp();
    } else if (RobotMap.driverController.getAButton()) {
      Intake.intakeMid();
    }
  }

  @Override
  public void exit(State nextState) {

  }
}