/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.rotate;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class HoldAngleWhileDriving extends CommandBase {
  private double angle;
  private boolean isFieldOriented;

  /**
   * Creates a new RotateToAngle.
   */
  public HoldAngleWhileDriving() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.resetPid();
    angle = RobotContainer.swerveDrive.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xSpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.xboxController.getY(Hand.kLeft)) * SwerveDrive.kMaxSpeed;

    var ySpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.xboxController.getX(Hand.kLeft)) * SwerveDrive.kMaxSpeed;

    isFieldOriented = (!RobotContainer.xboxController.getBumper(Hand.kLeft));

    RobotContainer.swerveDrive.holdAngleWhileDriving(xSpeed, ySpeed, angle, isFieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
