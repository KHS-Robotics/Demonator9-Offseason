/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Hood extends SubsystemBase {
  private CANSparkMax hood;
  private CANPIDController hoodPid;
  private CANEncoder hoodEnc;

  private double hoodPidSetpoint;
  /**
   * Creates a new Hood.
   */
  public Hood() {
    hood = new CANSparkMax(RobotMap.HOOD, MotorType.kBrushless);

    hoodPid = hood.getPIDController();
    hoodEnc = hood.getEncoder();

    setHoodPid(Constants.HOOD_P, Constants.HOOD_I, Constants.HOOD_D);

    hood.setIdleMode(IdleMode.kBrake);

    var tab = Shuffleboard.getTab("Hood");
    tab.addNumber("Hood Position", hoodEnc::getPosition);
    tab.addNumber("Hood Setpoint", () -> hoodPidSetpoint);
    tab.addNumber("Hood Error", () -> hoodPidSetpoint - hoodEnc.getPosition());
  }

  public void setHoodPid(double p, double i, double d) {
    hoodPid.setOutputRange(-0.35, 0.6);
    hoodPid.setP(p);
    hoodPid.setI(i);
    hoodPid.setD(d);
  }

  public void resetEnc() {
    hoodEnc.setPosition(0);
  }

  public void stop() {
    hood.set(0.0);
  }

  public void moveHood(double speed) {
    hood.set(speed);
  }

  public void setHood(double angle) {
    hoodPid.setReference(angle, ControlType.kPosition);
    hoodPidSetpoint = angle;
  }

  public double getPosition() {
    return hoodEnc.getPosition();
  }

  public void hoodMode(boolean brake) {
    if(brake) {
      hood.setIdleMode(IdleMode.kBrake);
    } else {
      hood.setIdleMode(IdleMode.kCoast);
    }
  }

  public boolean atSetpoint() {
    return Math.abs(hoodPidSetpoint - getPosition()) < 0.05;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
