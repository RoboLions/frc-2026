// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachines.scoring;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.Hood;
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
            ScoringStateMachine.hubAimState));
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
  }

  @Override
  public void init(State prevState) {
    System.out.println("HI im in idle state");
  }

  @Override
  public void execute() {
    Hood.simulateTurretAngle(Swerve.getPose(), Constants.Hood.TARGET_POSE, Swerve.getYawAsRadians(), Swerve.getYawRateAsRad()); 
    // this is the function that moves the turret
  }

  @Override
  public void exit(State nextState) {

  }
}