// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Helpers.ClimberState;
import static frc.robot.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private ClimberState state;
  private TalonFX rollers;
  private TalonFX pivot;
  private double startPos;
  private double distFromTarget;
  public Climber() {
    state = ClimberState.STOWED;
    rollers = new TalonFX(ROLLERID, "Team 3045");
    pivot = new TalonFX(PIVOTID, "Team 3045");
    startPos = pivot.getRotorPosition().getValueAsDouble();
    distFromTarget = 0;
  }

  public Command MoveOut() {
    return this.runOnce(() -> state = ClimberState.MOVINGOUT);
  }

  public Command Climb() {
    return this.runOnce(() -> state = ClimberState.MOVINGIN);
  }

  public void clearWay() {
    state = ClimberState.CLEARINGWAY;
  }

  public Command ClearWay() {
    return this.runOnce(() -> state = ClimberState.CLEARINGWAY);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Climber/State", state.toString());
    switch (state) {
      case MOVINGOUT:
        rollers.set(ROLLERSPEED);
        pivot.set(PIVOTSPEED);
        break;
      case MOVINGIN:
        distFromTarget = startPos - pivot.getRotorPosition().getValueAsDouble();
        SmartDashboard.putNumber("Climber/DistFromTarget", distFromTarget);
        if (Math.abs(distFromTarget) < ROTTOLLERANCE) {
          state = ClimberState.CLIMBED;
        }
        distFromTarget /= DISTDIVISOR;
        pivot.set(distFromTarget);
        rollers.set(HOLDSPEED);
        break;
      case STOWED:
        pivot.set(0);
        rollers.set(0);
        break;
      case CLIMBED:
        pivot.set(0);
        rollers.set(HOLDSPEED);
        break;
      case CLEARINGWAY:
        distFromTarget = 30 - pivot.getRotorPosition().getValueAsDouble();
        SmartDashboard.putNumber("Climber/DistFromTarget", distFromTarget);
        if (Math.abs(distFromTarget) < ROTTOLLERANCE) {
          state = ClimberState.CLIMBED;
        }
        distFromTarget /= DISTDIVISOR;
        pivot.set(distFromTarget);
        rollers.set(HOLDSPEED);
        break;
    }
  }
}
