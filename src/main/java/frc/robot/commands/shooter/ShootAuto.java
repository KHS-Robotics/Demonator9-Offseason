/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import frc.robot.RobotContainer;

public class ShootAuto extends Shoot {
  /**
   * Creates a new ShootAuto.
   */
  public ShootAuto(DoubleSupplier speed) {
    super(speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.indexer.getNumBalls() <= 0;
  }
}
