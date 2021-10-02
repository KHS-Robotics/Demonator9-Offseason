/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.drive.rotate.RotateToTarget;
import frc.robot.commands.hood.AlignHoodToTarget;
import frc.robot.commands.indexer.SetIndexerAuto;
import frc.robot.commands.shooter.RampShooter;
import frc.robot.commands.shooter.ShootAuto;

/**
 * Class to generate commands for autonomous movement
 */
public class AutoCommands {
  private static boolean initialized = false;
  //public static SwerveControllerCommand wallLineUp, frontTrench, pickTrench, returnTrench, moveOffInit, steal, moveFromSteal, pick3Rendevous, shootFromRendevous;
  public static SwerveControllerCommand driveOffLine, driveOffRotate, steal2;
  public static TrajectoryConfig config = new TrajectoryConfig(3.0, 6.0);

  public static void autoInit() {
    if(!initialized) {
      new Thread(() -> {
        //driveOffLine = loadPathweaverTrajectory("DriveOffLine");
        steal2 = loadPathweaverTrajectory("Steal2");

        driveOffLine = generatePath(new Pose2d(9, 5.2, Rotation2d.fromDegrees(0)), new Pose2d(6, 5.1, Rotation2d.fromDegrees(0)));
        driveOffRotate = generatePath(new Pose2d(9, 5.2, Rotation2d.fromDegrees(0)), new Pose2d(6, 5.1, Rotation2d.fromDegrees(180)));
        
      }).start();

      initialized = true;
    }
  }
    
  public static SwerveControllerCommand loadPathweaverTrajectory(String json) {
      Trajectory trajectory;
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + json + ".wpilib.json");
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + json, ex.getStackTrace());
        return null;
      }

      return new SwerveControllerCommand(
        trajectory,
        RobotContainer.swerveDrive::getPose,
        RobotContainer.swerveDrive.kinematics,
        // Position controllers
        new PIDController(0.90, 0.001, 0.30), // x (forward/backwards)
        new PIDController(0.90, 0.001, 0.30), // y (side to side)
        new ProfiledPIDController(1.75, 0.001, 0.40, new TrapezoidProfile.Constraints(Math.PI, Math.PI)), // theta (rotation)
        RobotContainer.swerveDrive::setModuleStates,
        RobotContainer.swerveDrive
      );
  }

  public static SwerveControllerCommand generatePath(Pose2d start, Pose2d end) {
    return generatePath(start, new Translation2d[] {}, end);
  }

  public static SwerveControllerCommand generatePath(Pose2d start, Translation2d[] mid, Pose2d end) {
    ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();

    for(int i = 0; i < mid.length; i++) {
      interiorWaypoints.add(mid[i]);
    }

    return getCommandFromTrajectory(TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config));
  }


  
  public static SwerveControllerCommand getCommandFromTrajectory(Trajectory trajectory) {
    return new SwerveControllerCommand(
        trajectory,
        RobotContainer.swerveDrive::getPose,
        RobotContainer.swerveDrive.kinematics,
        // Position controllers
        new PIDController(0.90, 0.001, 0.30), // x (forward/backwards)
        new PIDController(0.90, 0.001, 0.30), // y (side to side)
        new ProfiledPIDController(1.75, 0.001, 0.40, new TrapezoidProfile.Constraints(Math.PI, Math.PI)), // theta (rotation)
        RobotContainer.swerveDrive::setModuleStates,
        RobotContainer.swerveDrive
      );
  }


  public static Command DriveOffLine() {
    return
      driveOffLine;
  }

  public static Command DriveOffThenShoot() {
    return
      driveOffRotate
      .andThen(new RotateToTarget().alongWith(new AlignHoodToTarget()).alongWith(new RampShooter(() -> -3000)))
      .andThen(new ShootAuto(() -> -3000).alongWith(new SetIndexerAuto(0.65, () -> -3000)).alongWith(new RotateToTarget()).withTimeout(5));
  }

  public static Command Steal2() {
    return
      steal2;
  }
}