/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
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

public class TargetPIDTuner extends CommandBase {
  double p, i, d, setPoint;
  ShuffleboardTab tab;

  private NetworkTableEntry pVal, iVal, dVal, setPointEntry;

  public TargetPIDTuner() {
    this.addRequirements(RobotContainer.swerveDrive);

    tab = Shuffleboard.getTab("Swervedrive");
    pVal = tab.add("P-Value", 0).getEntry();
    iVal = tab.add("I-Value", 0).getEntry();
    dVal = tab.add("D-Value", 0).getEntry();

    setPointEntry = tab.add("Setpoint (Angle)", 0).getEntry();
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    p = pVal.getDouble(0.0);
    i = iVal.getDouble(0.0);
    d = dVal.getDouble(0.0);

    setPoint = setPointEntry.getDouble(0.0);

    RobotContainer.swerveDrive.setPID(p, i, d);
    RobotContainer.swerveDrive.rotateToAngleInPlace(setPoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stop();
  }
}
