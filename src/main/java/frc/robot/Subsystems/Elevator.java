// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private double elevatorHeight;
  private double numRotations;

  private TalonFX elevatorMotor1 = new TalonFX(MOTOR1ID, "Team3045");
  private TalonFX elevatorMotor2 = new TalonFX(MOTOR2ID, "Team3045");
  
  public Elevator() {
    numRotations = 0;
    elevatorHeight = 0;
  }

  private double calcHeightFromRotations(double pinnionRotations) {
    return pinnionRotations * PINNIONTODRUMRATIO * DRUMROTATIONTODISTANCE;
  }

  @Override
  public void periodic() {
    numRotations = elevatorMotor1.getRotorPosition().getValueAsDouble();
    elevatorHeight = calcHeightFromRotations(numRotations);
    // This method will be called once per scheduler run
  }
}
