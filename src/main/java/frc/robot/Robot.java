// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.statemachines.drivetrain.DrivetrainStateMachine;
import frc.robot.subsystems.statemachines.scoring.ScoringStateMachine;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else if (isSimulation()) {
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    RobotMap.init();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    RobotMap.subsystemPeriodics();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().run(); // we will always need this in auto, command schedulers are not built into the LoggedRobot class, but it is for TimedRobot.
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run(); // we will always need this in auto, command schedulers are not built into the LoggedRobot class, but it is for TimedRobot.
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    RobotMap.scoringStateMachine.setCurrentState(ScoringStateMachine.idleState);
    RobotMap.drivetrainStateMachine.setCurrentState(DrivetrainStateMachine.teleopState);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    RobotMap.drivetrainStateMachine.setNextState();
    RobotMap.scoringStateMachine.setNextState();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
