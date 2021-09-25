/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.pid;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterPIDTuner extends CommandBase {
  private NetworkTableEntry pVal, iVal, dVal, setPointEntry;
  double p, i, d, setPoint;
  ShuffleboardTab tab;

  /**
   * Creates a new ShooterPIDTuner.
   */
  public ShooterPIDTuner() {
    addRequirements(RobotContainer.shooter);
    tab = Shuffleboard.getTab("Shooter");
    pVal = tab.add("Shooter P-Value", 0).getEntry();
    iVal = tab.add( "Shooter I-Value", 0).getEntry();
    dVal = tab.add("Shooter D-Value", 0).getEntry();

    setPointEntry = tab.add("Shooter Setpoint", 0).getEntry();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    p = pVal.getDouble(0.0);
    i = iVal.getDouble(0.0);
    d = dVal.getDouble(0.0);

    setPoint = setPointEntry.getDouble(0.0);

    RobotContainer.shooter.setShooterPidF(p, i, d, 1.0 / 6000.0);
    RobotContainer.shooter.setShooter(setPoint);
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
