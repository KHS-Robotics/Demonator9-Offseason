/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.vision.Limelight.LightMode;
import frc.robot.commands.CenterSwerveModules;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.vision.Limelight;


public class Robot extends TimedRobot {
  int id = 1;
  static int homeId = 0;
  NetworkTableEntry idChooser;

  RobotContainer robotContainer;

  Command autonCommand;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    AutoCommands.autoInit();

    var matchTab = Shuffleboard.getTab("Match");
    idChooser = matchTab.add("Auto ID", 1).getEntry();
    matchTab.addNumber("Auto ID Echo", () -> id);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    Limelight.setLedMode(LightMode.eOff);

    RobotContainer.hood.hoodMode(false);
    RobotContainer.CPManipulator.brakeMode(false);
    SwerveDrive.kMaxSpeed = 3.5;
    SwerveDrive.kMaxAngularSpeed = Math.PI;
  }

  @Override
  public void disabledPeriodic() {    
    if(RobotContainer.xboxController.getBButton() && RobotContainer.xboxController.getAButton() && RobotContainer.xboxController.getBumper(Hand.kLeft)) {
      RobotContainer.hood.resetEnc();
      RobotContainer.hood.stop();
    }

    id = (int) idChooser.getDouble(1);

    if(RobotContainer.xboxController.getXButtonPressed()) {
      Limelight.setLedMode(LightMode.eOn);
    } else if(RobotContainer.xboxController.getYButtonPressed()) {
      Limelight.setLedMode(LightMode.eOff);
    }

  }

  @Override
  public void autonomousInit() {
    if(autonCommand != null) {
      autonCommand.cancel();
      CommandScheduler.getInstance().run();
    }

    Limelight.setLedMode(LightMode.eOn);
    RobotContainer.hood.hoodMode(true);
    RobotContainer.CPManipulator.brakeMode(true);

    RobotContainer.swerveDrive.resetNavx(robotContainer.getStartingPose(id));

    //RobotContainer.shooter.setShooter(-3000);
    RobotContainer.intake.intake();
    
    Command putIntakeDown = new InstantCommand(() -> RobotContainer.intake.down())
    .andThen(new WaitCommand(.5)
    .andThen(() -> RobotContainer.intake.setOff()));

    RobotContainer.indexer.setNumBalls(0);

    Command desiredAuton = 
      robotContainer.getAutonomousCommand(id)
      .alongWith(putIntakeDown)
      .andThen(() -> {
        RobotContainer.swerveDrive.stop();
        RobotContainer.shooter.stop();
        RobotContainer.hood.stop();
        RobotContainer.indexer.stop();
        RobotContainer.intake.stop();
    }, RobotContainer.swerveDrive);//, RobotContainer.shooter, RobotContainer.hood, RobotContainer.indexer, RobotContainer.intake);

    if(!CenterSwerveModules.hasCalibrated()) {
      autonCommand = new CenterSwerveModules().andThen(desiredAuton);
    } else {
      autonCommand = desiredAuton;
    }

    if(autonCommand != null) {
      autonCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    RobotContainer.swerveDrive.updateOdometry();
  }

  @Override
  public void teleopInit() {
    Limelight.setLedMode(LightMode.eOff);
    RobotContainer.hood.hoodMode(true);
    RobotContainer.CPManipulator.brakeMode(true);
    RobotContainer.shooter.stop();
    RobotContainer.intake.stop();

    if(autonCommand != null) {
      autonCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    RobotContainer.swerveDrive.updateOdometry();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.intake.stop();
  }

  @Override
  public void testPeriodic() {
    
  }
}