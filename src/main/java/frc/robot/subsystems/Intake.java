/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax motor;
  private DoubleSolenoid solenoid;
  private double speed = 0.5; //.7 made belt slip off
  private CANEncoder motorEnc;

  public Intake() {
    motor = new CANSparkMax(RobotMap.INTAKE, MotorType.kBrushless);
    solenoid = new DoubleSolenoid(RobotMap.INTAKE_SOLENOID_1, RobotMap.INTAKE_SOLENOID_2);

    motorEnc = motor.getEncoder();

    motor.setIdleMode(IdleMode.kBrake);

    var tab = Shuffleboard.getTab("Intake");
    tab.addNumber("Velocity", motorEnc::getVelocity);

    setOff();
  }

  @Override
  public void periodic() {

  }

  public double getVelocity() {
    return motorEnc.getVelocity();
  }

  public void stop() {
		motor.set(0.0);
	}
	
	public void intake() {    
		motor.setVoltage(speed * 12);
  }
  
  public void intake(double speed) {
    motor.setVoltage(speed * 12);
  }

	public void reverse() {
		motor.setVoltage(-speed * 12);
	}

  public void down() {
    solenoid.set(Value.kReverse);
  } 

  public void up() {
    solenoid.set(Value.kForward);
  }

  public void setOff() {
    solenoid.set(Value.kOff);
  }
}
