/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SetIndexer extends CommandBase {
  double speed;
  DoubleSupplier shooterSpeed;
  /**
   * Creates a new SetIndexer.
   */
  public SetIndexer(double speed, DoubleSupplier shooterSpeed) {
    addRequirements(RobotContainer.indexer, RobotContainer.intake);
    this.speed = speed;
    this.shooterSpeed = shooterSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.shooter.atSetpoint(shooterSpeed.getAsDouble())) {
      RobotContainer.indexer.setMotor(speed);

      if(RobotContainer.indexer.getNumBalls() < 4) {
        RobotContainer.intake.intake();
      }
      
    } else {
      RobotContainer.indexer.stop();
      RobotContainer.intake.stop();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.stop();
    RobotContainer.indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
