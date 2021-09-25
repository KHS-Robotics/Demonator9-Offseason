/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Shoot extends CommandBase {
  boolean drop = false;
  //final double THRESHOLD = 23.0;

  /**
   * Creates a new shoot.
   */

  DoubleSupplier speed;
  public Shoot(DoubleSupplier speed) {
    addRequirements(RobotContainer.shooter);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter.setShooter(speed.getAsDouble());
    drop = false;
    //RobotContainer.indexer.setMotor(.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(RobotContainer.shooter.getCurrent() > THRESHOLD && !drop) {
    //   drop = true;
    //   RobotContainer.indexer.decrementBall();
    // } else if(drop && RobotContainer.shooter.getCurrent() < THRESHOLD) {
    //   drop = false;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
