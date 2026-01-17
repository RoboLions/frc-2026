// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachines.scoring;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.Hood;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.swerve.Swerve;

/** Add your docs here. */
public class IntakeState extends State {

  @Override
  public void build() {
    addTransition(
        new Transition(
            () -> {
              return RobotMap.driverController.getLeftBumper();
            },
            ScoringStateMachine.outtakeState));
  

      addTransition(
        new Transition(
            () -> {
              return RobotMap.driverController.getBButton();
            },
            ScoringStateMachine.idleState));
  }
  @Override
  public void init(State prevState) {
    Intake.simulateIntakeDown();
  }

  @Override
  public void execute() {
    Hood.simulateTurretAngle(Swerve.getPose(), Constants.Hood.HUB_POSE, Swerve.getYawAsRadians(), Swerve.getYawRateAsRad());
  }

  @Override
  public void exit(State nextState) {}
}