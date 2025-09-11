// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private double elevatorHeight;
  private double numRotations;
  
  public Elevator() {}

  private double calcHeightFromRotations(double pinnionRotations) {
    return pinnionRotations * PINNIONTODRUMRATIO * DRUMROTATIONTODISTANCE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
