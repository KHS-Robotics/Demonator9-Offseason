/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import frc.robot.RobotContainer;

public class SetIndexerAuto extends SetIndexer {
  /**
   * Creates a new SetIndexerAuto.
   */
  public SetIndexerAuto(double speed, DoubleSupplier shootingSpeed) {
    super(speed, shootingSpeed);
  }

  @Override
  public void end(boolean interupted) {
    RobotContainer.indexer.stop();
    RobotContainer.intake.intake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.indexer.getNumBalls() <= 0;
  }
}
