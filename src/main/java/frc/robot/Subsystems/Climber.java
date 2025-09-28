// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Helpers.ClimberState;
import static frc.robot.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private ClimberState state;
  private TalonFX rollers;
  private TalonFX pivot;
  public Climber() {
    state = ClimberState.STOWED;
    rollers = new TalonFX(ROLLERID, "Team 3045");
    pivot = new TalonFX(PIVOTID, "Team 3045");
  }

  public Command MoveOut() {
    return this.runOnce(() -> state = ClimberState.MOVINGOUT);
  }

  public Command Climb() {
    return this.runOnce(() -> state = ClimberState.MOVINGIN);
  }

  @Override
  public void periodic() {
    switch (state) {
      case MOVINGOUT:
        rollers.set(ROLLERSPEED);
        pivot.set(PIVOTSPEED);
        break;
      case MOVINGIN:
        double distFromTarget = -pivot.getRotorPosition().getValueAsDouble();
        distFromTarget /= DISTDIVISOR;
        pivot.set(distFromTarget);
        rollers.set(HOLDSPEED);
        if (distFromTarget < ROTTOLLERANCE) {
          state = ClimberState.CLIMBED;
        }
        break;
      case STOWED:
        pivot.set(0);
        rollers.set(0);
        break;
      case CLIMBED:
        pivot.set(0);
        rollers.set(HOLDSPEED);
        break;
    }
  }
}
