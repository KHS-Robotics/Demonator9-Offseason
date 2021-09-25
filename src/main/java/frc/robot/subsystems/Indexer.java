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
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Indexer extends SubsystemBase {
  private final double MAX_VEL = 10000;
  private volatile int numBalls = 0;
  private CANSparkMax motor;
  private CANPIDController motorPid;
  private CANEncoder motorEnc;
  private DigitalInput input1, input2, input3, input5, inputDecrement;

  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    motor = new CANSparkMax(RobotMap.INDEX_MOTOR, MotorType.kBrushless);
    motorPid = motor.getPIDController();
    motorEnc = motor.getEncoder();

    motorEnc.setVelocityConversionFactor(1);
    motorEnc.setPositionConversionFactor(1);

    motorEnc.setPosition(0);

    motorPid.setIZone(200);
    motorPid.setOutputRange(-1, 1);

    motorPid.setP(Constants.INDEXER_P);
    motorPid.setI(Constants.INDEXER_I);
    motorPid.setD(Constants.INDEXER_D);
    motorPid.setFF(Constants.INDEXER_FF);

    motorPid.setSmartMotionMaxAccel(10000, 0);
    motorPid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    motorPid.setSmartMotionMaxVelocity(MAX_VEL, 0);
    // motorPid.setSmartMotionMinOutputVelocity(100, 0);
    motorPid.setSmartMotionAllowedClosedLoopError(0.5, 0);

    motor.setIdleMode(IdleMode.kBrake);

    input1 = new DigitalInput(RobotMap.LIMIT_PORT_1);
    input2 = new DigitalInput(RobotMap.LIMIT_PORT_2);
    input3 = new DigitalInput(RobotMap.LIMIT_PORT_3);
    input5 = new DigitalInput(RobotMap.LIMIT_PORT_5);
    inputDecrement = new DigitalInput(RobotMap.LIMIT_PORT_DECREMENT);

    var tab = Shuffleboard.getTab("Indexer");
    tab.addNumber("Motor Speed", motorEnc::getVelocity);
    // tab.addNumber("Position", motorEnc::getPosition);
    tab.addNumber("Num Balls", () -> numBalls);
    tab.addBoolean("Beam Break 1", input1::get);
    tab.addBoolean("Beam Break 2", input2::get);
    tab.addBoolean("Beam Break 3", input3::get);
    // tab.addBoolean("Beam Break 4", input4::get);
    tab.addBoolean("Beam Break 5", input5::get);

    var matchTab = Shuffleboard.getTab("Match");
    matchTab.addNumber("Num Balls", this::getNumBalls);
    matchTab.addBoolean("5 Balls", () -> getNumBalls() >= 5);

    new Thread(() -> {
      boolean wasRead = false;
      while (true) {
        if (inputDecrement.get() && !wasRead) {
          wasRead = true;
        } else if (!inputDecrement.get() && wasRead) {
          decrementBall();
          wasRead = false;
        }
        try {
          Thread.sleep(5);
        } catch (InterruptedException e) {
        }
      }
    }).start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotor(double speed) {
    if(speed > 0) {
      motorPid.setReference(speed * MAX_VEL, ControlType.kVelocity);
    } else {
      motor.set(speed);
    }
  }

  public void stop() {
    motor.set(0);
  }

  public boolean getSwitch1() {
    return !input1.get();
  }

  public boolean getSwitch2() {
    return !input2.get();
  }

  public boolean getSwitch3() {
    return !input3.get();
  }

  public boolean getSwitch5() {
    return !input5.get();
  }

  public boolean getDecrementSwitch() {
    return !inputDecrement.get();
  }

  public double getPosition() {
    return motorEnc.getPosition();
  }

  public void setPosition(double position) {
    motorPid.setReference(position, ControlType.kSmartMotion);
  }

  public boolean atSetpoint(double setpoint) {
    return Math.abs(motorEnc.getPosition() - setpoint) < .5;
  }

  public void setNumBalls(int num) {
    if(num >= 0 && num <= 5) {
      numBalls = num;
    }
  }

  public void incrementBall() {
    numBalls++;
    if(numBalls > 5) {
      numBalls = 5;
    }
  }

  public void decrementBall() {
    numBalls--;
    if(numBalls < 0) {
      numBalls = 0;
    }
  }

  public int getNumBalls() {
    return numBalls;
  }

  public void zeroBalls() {
    numBalls = 0;
  }
}
