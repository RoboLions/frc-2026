package frc.robot.subsystems.statemachines.scoring;

import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.Shooter;
import frc.robot.subsystems.interfaces.swerve.Swerve;
import frc.robot.subsystems.interfaces.LED;

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
    addTransition(
        new Transition(
            () -> {
              return RobotMap.driverController.getRightBumperButtonPressed();
            },
            ScoringStateMachine.intakeState));
  }

  @Override
  public void init(State prevState) {
    Intake.intakeDown();
    LED.setFlashRed();
  }

  @Override
  public void execute() {
    if (RobotMap.driverController.getRightTriggerAxis() > 0.25) { // SHOOT
      Intake.FeedIn();
      Intake.IndexIn();
      LED.setFlashGreen();
    }

    if (RobotMap.driverController.getLeftBumperButtonPressed() || RobotMap.manipulatorController.getLeftBumperButtonPressed()) {
      Intake.intakeMid();
      Intake.intakeSlow();
    } else if (RobotMap.driverController.getRightBumperButtonPressed() || RobotMap.driverController.getAButtonPressed() || RobotMap.manipulatorController.getRightBumperButtonPressed()) {
      Intake.intakeDown();
    }

    if (Swerve.getPose().getX() >= 4.75 && Swerve.getPose().getX() <= 11.75) { // PASS LOGIC VS HUB
      
      if (Swerve.getPose().getY() > 4) {
       Shooter.interpoleateAndPass(Swerve.getPose().getTranslation().getDistance(Constants.FIELD.PASS_UPPER.toTranslation2d()));
      } else {
       Shooter.interpoleateAndPass(Swerve.getPose().getTranslation().getDistance(Constants.FIELD.PASS_LOWER.toTranslation2d()));
      }
      
      isPass = true;
    } else {
      Shooter.interpolateAndShoot(Swerve.getPose().getTranslation().getDistance(Constants.FIELD.HUB_POSE));
      isPass = false;
    }
  }

  @Override
  public void exit(State nextState) {
    isPass = false;
    LED.turnOff();

  }
}
