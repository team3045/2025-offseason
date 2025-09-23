// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private double elevatorHeight;
  private double numRotations;
  private double targetHeight;
  private double targetRotations;
  private double motorZero;

  private TalonFX elevatorMotor1;
  private TalonFX elevatorMotor2;
  private AlgaeIntake algaeIntake;
  
  public Elevator(AlgaeIntake AlgaeIntake) {
    numRotations = 0;
    elevatorHeight = 0;
    targetHeight = 0;
    targetRotations = 0;
    elevatorMotor1 = new TalonFX(MOTOR1ID, "Team 3045");
    elevatorMotor2 = new TalonFX(MOTOR2ID, "Team 3045");
    algaeIntake = AlgaeIntake;
    motorZero = elevatorMotor1.getRotorPosition().getValueAsDouble();
  }

  private double calcHeightFromRotations(double pinnionRotations) {
    return pinnionRotations * PINNIONTODRUMRATIO * DRUMROTATIONTODISTANCE;
  }

  private double calcRotationsFromHeight(double height) {
    return height * (1/PINNIONTODRUMRATIO) * (1/DRUMROTATIONTODISTANCE);
  }

  public boolean atTargetHeight() {
    return Math.abs(elevatorHeight - targetHeight) <= HEIGHTTOLLERANCE;
  }

  public void goToHeight(double TargetHeight) {
    targetHeight = TargetHeight;
  }

  @Override
  public void periodic() {
    numRotations = elevatorMotor1.getRotorPosition().getValueAsDouble();
    elevatorHeight = calcHeightFromRotations(numRotations);
    targetRotations = calcRotationsFromHeight(targetHeight) + motorZero;
    double rotDiff = numRotations - targetRotations;
    double speed = rotDiff/SPEEDDOWN;
    SmartDashboard.putNumber("Elevator/RotDiff", rotDiff);
    SmartDashboard.putNumber("Elevator/Speed", speed);
    SmartDashboard.putNumber("Elevator/TargetRot", targetRotations);
    SmartDashboard.putNumber("Elevator/CurrRot", numRotations);
    if (Math.abs(rotDiff) > 0.2) {
      elevatorMotor1.set(Math.min(speed, 0.5));
      elevatorMotor2.set(Math.min(speed, 0.5));
    } else {
      elevatorMotor1.set(HOLDSPEED);
      elevatorMotor2.set(HOLDSPEED);
    }
    if (elevatorHeight < LOWERCLEARENCEBOUND && elevatorHeight > UPPERCLEARENCEBOUND) {
      algaeIntake.moveDown();
    } else {
      algaeIntake.moveUp();
    }
  }
}