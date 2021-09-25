/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.vision.Limelight;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final double LIMELIGHT_ANGLE = 20.0; // 20 Degree Tilt
  
  private CANSparkMax leader, follower;
  private CANPIDController shooterPid;
  private CANEncoder leaderEnc;

  private double shooterPidSetpoint, rpmMultiplier = 1;
  private boolean isClimbing, coastMode;

  public Shooter() {
    leader = new CANSparkMax(RobotMap.SHOOTER1, MotorType.kBrushless);
    follower = new CANSparkMax(RobotMap.SHOOTER2, MotorType.kBrushless);

    follower.follow(leader);
    shooterPid = leader.getPIDController();

    leaderEnc = leader.getEncoder();

    setShooterPidF(Constants.SHOOTER_P, Constants.SHOOTER_I, Constants.SHOOTER_D, Constants.SHOOTER_FF);

    shooterPid.setIZone(500);
    shooterPid.setOutputRange(-1, 0);

    leader.setIdleMode(IdleMode.kCoast);
    follower.setIdleMode(IdleMode.kCoast);
    coastMode = true;
    
    var tab = Shuffleboard.getTab("Shooter");
    tab.addNumber("Leader Speed", leaderEnc::getVelocity);
    tab.addNumber("Shooter Setpoint", () -> shooterPidSetpoint);
    tab.addNumber("Shooter Error", () -> shooterPidSetpoint - leaderEnc.getVelocity());
    tab.addBoolean("Is Climbing", () -> isClimbing);
    tab.addNumber("Current", () -> getCurrent());
    tab.addNumber("Multiplier", this::getRPMMultipler);
    tab.addBoolean("PTO Switch", () -> RobotContainer.switchbox.engagePTO());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooter(double speed) {
    if(!coastMode) {
      setBrake(true);
    }

    shooterPidSetpoint = speed * getRPMMultipler();
    shooterPid.setReference(shooterPidSetpoint, ControlType.kVelocity);
  }

  public void setShooterWithoutPID(double speed) {
    leader.set(speed);
  }

  public double getSetpoint() {
    return shooterPidSetpoint;
  }

  public void stop() {
    leader.set(0.0);
    isClimbing = false;
  }

  public void stopShooter() {
    leader.set(0.0);
  }

  public void setShooterPidF(double p, double i, double d, double ff) {
    shooterPid.setP(p);
    shooterPid.setI(i);
    shooterPid.setD(d);
    shooterPid.setFF(ff);
  }

  public void setBrake(boolean brake) {
    leader.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    follower.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    coastMode = !brake;
  }

  public void enableForClimb() {
    isClimbing = true;
    leader.set(1);
    setBrake(true);
  }

  public void disableForClimb() {
    isClimbing = false;
    leader.set(0.0);
  }

  public boolean canEngagePTO() {
    return leaderEnc.getVelocity() > -100;
  }

  public double getVertAngle() {
    return LIMELIGHT_ANGLE + Limelight.getTy();
  }

  public double getCurrent() {
    return RobotContainer.pdp.getCurrent(12);
  }

  public double getRPMMultipler() {
    return rpmMultiplier;
  }

  public void increaseRPM() {
    rpmMultiplier += 0.05;
  }

  public void decreaseRPM() {
    rpmMultiplier -= 0.05;    
  }

  public boolean atSetpoint(double speed) {
    return Math.abs(speed - leaderEnc.getVelocity()) < 200;
  }
}
