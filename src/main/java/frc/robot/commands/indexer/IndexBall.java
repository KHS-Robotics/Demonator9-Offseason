/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

public class IndexBall extends CommandBase {
  private static boolean isDone = true, toggled = false;
  long startTime;
  
  /**
   * Creates a new IndexBall.
   */
  public IndexBall() {
    addRequirements(RobotContainer.indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    isDone = false;
    toggled = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = .45;
 
    if (RobotContainer.indexer.getNumBalls() == 1 && !RobotContainer.indexer.getSwitch2()) {
      RobotContainer.indexer.setMotor(speed);
    } else if (RobotContainer.indexer.getNumBalls() > 1 && (RobotContainer.indexer.getSwitch1() || !RobotContainer.indexer.getSwitch2())) {
      RobotContainer.indexer.setMotor(speed);
    } else {
      isDone = true;
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.indexer.stop();
    isDone = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone || (RobotContainer.indexer.getNumBalls() > 4);
  }

  public static boolean isIndexing() {
    return !isDone;
  }
}