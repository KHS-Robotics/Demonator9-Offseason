/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */

  private CANSparkMax telescope;
  //private CANEncoder telescopeEnc;
  private Solenoid pto;
  
  public Climber() {
    telescope = new CANSparkMax(RobotMap.TELESCOPE, MotorType.kBrushless);
    pto = new Solenoid(RobotMap.PTO);
    setPTO(false);

    telescope.setIdleMode(IdleMode.kBrake);

    var tab = Shuffleboard.getTab("Climber");
    // tab.addNumber("Raise Speed", () -> telescopeEnc.getVelocity());
    // tab.addNumber("Position", () -> telescopeEnc.getPosition());
    tab.addBoolean("PTO Engaged", pto::get);

    var matchTab = Shuffleboard.getTab("Match");
    matchTab.addBoolean("Climb", pto::get);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTelescope(double speed) {
    telescope.set(speed);
  }

  public void setPTO(boolean climbing) {
    pto.set(climbing);
  }
}
