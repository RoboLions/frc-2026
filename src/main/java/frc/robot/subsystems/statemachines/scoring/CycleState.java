package frc.robot.subsystems.statemachines.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.Shooter;
import frc.robot.subsystems.interfaces.LED;
import frc.robot.subsystems.interfaces.swerve.Swerve;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

public class CycleState extends State {

  public static boolean isPass = false;
  private static Pose2d HUB_POSE = new Pose2d(Constants.FIELD.HUB_POSE, new Rotation2d());

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
    LED.setFlashBlue();
  }

  @Override
  public void execute() {
    if (RobotMap.driverController.getRightTriggerAxis() > 0.25) { // SHOOT
      Intake.FeedIn();
    }

    if (RobotMap.driverController.getLeftBumperButton() || RobotMap.manipulatorController.getLeftBumperButton()) { // INTAKE UP AND DOWN
      Intake.intakeMid();
    } else {
      Intake.intakeDown();
    }

    if (Swerve.getPose().getX() >= 4.75 && Swerve.getPose().getX() <= 11.75) { // PASS LOGIC VS HUB
      LED.setFlashRed();
      isPass = true;
    } else {
      Shooter.interpolateAndShoot(Swerve.getDistToPose(HUB_POSE));
      LED.setFlashGreen();
      isPass = false;
    }
  }

  @Override
  public void exit(State nextState) {
    isPass = false;
  }
}
