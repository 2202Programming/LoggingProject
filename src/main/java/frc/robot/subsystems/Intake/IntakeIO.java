// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.Intake_Constants;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs{  
    public static double IntakeMotorStrength = Intake_Constants.IntakeMotorStrength;
    public static double CarwashMotorStrength = Intake_Constants.CarwashMotorStrength;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void motor_config(CANSparkMax mtr, boolean inverted) {}
  
   //Turn Intake Motor On by sending a double value
  public default void setIntakeSpeed(double intakeMotorStrength) {}

  public default void intakeOff() {}

  //Deploy arm mechanism using a Double Solenoids
  public default void deploy() {}

  //Retract arm mechanism using a Double Solenoids
  public default void retract() {}
  
  public default void setCarwashSpeed(double carwashMotorStrength) {}

  public default void carwashOn() {}

  public default void carwashOnReverse() {}

  public default void carwashOff() {}

  public default void carwashReverse() {}

  /**
   * NTs
   */

  NetworkTable nt = NetworkTableInstance.getDefault().getTable("Intake");
  NetworkTableEntry nt_intakeSpeed = nt.getEntry("Intake Speed");
  NetworkTableEntry nt_carwashSpeed = nt.getEntry("Carwash Speed");

  public default void ntconfig() {
    // nt_intakeSpeed.setDouble(0.0);
    // nt_carwashSpeed.setDouble(0.0);
  }

  public default void ntupdates() {
    // if (nt_intakeSpeed.getDouble(0.0) != IntakeMotorStrength) setIntakeSpeed(nt_intakeSpeed.getDouble(0.0));
    // if (nt_carwashSpeed.getDouble(0.0) != CarwashMotorStrength) setIntakeSpeed(nt_carwashSpeed.getDouble(0.0));
  }
}
