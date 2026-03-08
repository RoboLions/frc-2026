package frc.robot.subsystems.statemachines.scoring;

import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.Shooter;
import frc.robot.subsystems.interfaces.Turret;
import frc.robot.subsystems.interfaces.Turret.SimulationObjects;
import frc.robot.subsystems.swerve.Swerve;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

public class CycleState extends State {

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
  public void init(State prevState) {
    Shooter.setShootSpeed(SimulationObjects.totalShotVelocity);
    Intake.intakeDown();
    Intake.intake();
  }

  @Override
  public void execute() {
    if (RobotMap.driverController.getRightBumperButton() && !(Turret.getAzimuthAngularVelocity() > 20)) {
      Intake.allRollersIn();
    }

    if (RobotMap.driverController.getAButton()) {
      Intake.intakeMid();
    } else {
      Intake.intakeDown();
    }

    if (Swerve.getPose().getX() >= 4.75 && Swerve.getPose().getX() <= 11.75) {
      Turret.turretTrackPassPose();
    } else {
      Turret.turretTrackHub();
    }

    Shooter.setShootSpeed(SimulationObjects.totalShotVelocity);
  }

  @Override
  public void exit(State nextState) {}
}
