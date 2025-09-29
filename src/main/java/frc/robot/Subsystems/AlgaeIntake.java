// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static frc.robot.Constants.AlgaeIntakeConstants.*;
import static frc.robot.Constants.EndEffectorConstants.ALGAEINTAKEANGLE;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Helpers.HandlerState;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  private HandlerState state;
  private HandlerState prevState;
  private double timeStarted;
  private double time;
  private TalonFX pivot;
  private TalonFX roller;
  private CANrange canRange;
  private Supplier<Distance> rangeSupplier;
  private EndEffector effector;
  private boolean isIntaking;
  private double startRot;

  public AlgaeIntake(EndEffector Effector) {
    roller = new TalonFX(ROLLERID, "Team 3045");
    pivot = new TalonFX(PIVOTID, "Team 3045");
    canRange = new CANrange(CANRANGEID, "Team 3045");
    effector = Effector;
    state = HandlerState.IDLE;
    prevState = HandlerState.IDLE;
    rangeSupplier = canRange.getDistance().asSupplier();
    timeStarted = Timer.getTimestamp();
    time = Timer.getTimestamp();
    isIntaking = false;
    startRot = 0;
  }

  public void intake() {
    timeStarted = Timer.getTimestamp();
    prevState = HandlerState.INTAKING;
    state = HandlerState.MOVINGDOWN;
    isIntaking = true;
  }

  public void outtake() {
    timeStarted = Timer.getTimestamp();
    prevState = HandlerState.OUTTAKING;
    state = HandlerState.MOVINGDOWN;
  }

  public void moveDown() {
    timeStarted = Timer.getTimestamp();
    prevState = HandlerState.IDLE;
    state = HandlerState.MOVINGDOWN;
  }

  public void moveUp() {
    timeStarted = Timer.getTimestamp();
    prevState = HandlerState.IDLE;
    state = HandlerState.MOVINGUP;
  }

  public void clearWay() {
    timeStarted = Timer.getTimestamp();
    prevState = HandlerState.IDLE;
    state = HandlerState.MOVINGUP;
    isIntaking = false;
    startRot = pivot.getRotorPosition().getValueAsDouble();
  }

  public Command Intake() {
    return this.runOnce(() -> intake());
  }

  public Command Outtake() {
    return this.runOnce(() -> outtake());
  }

  public Command MoveDown() {
    return this.runOnce(() -> moveDown());
  }

  public Command MoveUp() {
    return this.runOnce(() -> moveUp());
  }

  @Override
  public void periodic() {
    time = Timer.getTimestamp();
    SmartDashboard.putNumber("AlgaeIntake/Range", rangeSupplier.get().baseUnitMagnitude());
    SmartDashboard.putNumber("AlgaeIntake/PivotCurrent", Math.abs(pivot.getTorqueCurrent().getValueAsDouble()));
    SmartDashboard.putNumber("AlgaeIntake/PivotVelocity", Math.abs(pivot.getVelocity().getValueAsDouble()));
    SmartDashboard.putString("AlgaeIntake/IntakeState", state.toString());
    // This method will be called once per scheduler run
    pivot.setNeutralMode(NeutralModeValue.Brake);
    switch (state) {
      case MOVINGDOWN:
        pivot.set(-PIVOTSPEED);
        if (isIntaking) {
          if (Math.abs(pivot.getTorqueCurrent().getValueAsDouble()) > 20 && Math.abs(pivot.getVelocity().getValueAsDouble()) < 5) {
            state = prevState;
            pivot.set(0);
          }
        } else {
          if (Math.abs(pivot.getRotorPosition().getValueAsDouble() - startRot) >= TOTALROTATIONSFORCOLLISION) {
            state = prevState;
            pivot.set(0);
          }
        }
        break;
      case MOVINGUP:
        pivot.set(PIVOTSPEED);
        if (Math.abs(pivot.getTorqueCurrent().getValueAsDouble()) > 20 && Math.abs(pivot.getVelocity().getValueAsDouble()) < 5) {
          state = prevState;
          pivot.set(0);
        }
        break;
      case INTAKING:
        roller.set(ROLLERSPEED);
        effector.setRunning(true);
        effector.setDir(1);
        if ((time - timeStarted) >= INTAKELENGTHSECONDS) {
          state = HandlerState.IDLE;
        }
        if (rangeSupplier.get().baseUnitMagnitude() <= DEFAULTCANRANGEDIST) {
          state = HandlerState.IDLE;
        }
        break;
      case OUTTAKING:
        roller.set(-ROLLERSPEED);
        effector.setRunning(true);
        effector.setDir(-1);
        if ((time - timeStarted) >= INTAKELENGTHSECONDS) {
          state = HandlerState.IDLE;
        }
        break;
      case STOWED:
        roller.set(0);
        effector.setRunning(false);
        break;
      case IDLE:
        roller.set(0);
        effector.setRunning(false);
        prevState = HandlerState.STOWED;
        state = HandlerState.MOVINGUP;
        break;
    }
  }
}
