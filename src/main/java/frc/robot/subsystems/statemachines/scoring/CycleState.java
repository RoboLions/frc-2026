package frc.robot.subsystems.statemachines.scoring;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.Shooter;
import frc.robot.subsystems.interfaces.Turret;
import frc.robot.subsystems.interfaces.Turret.SimulationObjects;
import frc.robot.subsystems.interfaces.swerve.Swerve;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

public class CycleState extends State {

  public static boolean isPass = false;

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
    Intake.intakeDown();
    Intake.intake();
  }

  @Override
  public void execute() {
    RobotMap.driverController.setRumble(GenericHID.RumbleType.kRightRumble, 0.1);

    if (RobotMap.driverController.getRightTriggerAxis() > 0.25) { // SHOOT
      Intake.allRollersIn();
      RobotMap.driverController.setRumble(GenericHID.RumbleType.kRightRumble, 0.25);
    } else if (RobotMap.manipulatorController.getRightTriggerAxis() > 0.25) {
      Intake.FeedOut();
      Intake.IndexOut();
      RobotMap.driverController.setRumble(GenericHID.RumbleType.kRightRumble, 0.25);
    }

    if (RobotMap.driverController.getLeftBumperButton() || RobotMap.manipulatorController.getLeftBumperButton()) { // INTAKE UP AND DOWN
      Intake.intakeMid();
    } else {
      Intake.intakeDown();
    }

    if (Swerve.getPose().getX() >= 4.75 && Swerve.getPose().getX() <= 11.75) { // PASS LOGIC VS HUB
      Turret.turretTrackPassPose();
      isPass = true;
    } else {
      Turret.turretTrackHub();
      isPass = false;
    }

    Shooter.setShootSpeed(SimulationObjects.totalShotVelocity);
  }

  @Override
  public void exit(State nextState) {
    isPass = false;
  }
}
