/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.rotate;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.LightMode;
import frc.robot.RobotContainer;

public class RotateToTarget extends CommandBase {
  /**
   * Creates a new RotateToAngle.
   */
  public RotateToTarget() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.setLedMode(LightMode.eOn);

    RobotContainer.swerveDrive.resetPid();
    RobotContainer.swerveDrive.stop();
  }

  @Override
  public void execute() {
    RobotContainer.swerveDrive.rotateToAngleInPlace(RobotContainer.swerveDrive.getYaw() - Limelight.getTx() - 1);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.swerveDrive.atSetpoint();
  }
}
