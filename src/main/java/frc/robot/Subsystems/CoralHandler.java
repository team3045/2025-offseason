// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Helpers.HandlerState;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.CoralIntakeConstants.*;

import java.util.function.Supplier;

public class CoralHandler extends SubsystemBase {
  /** Creates a new CoralHandler. */
  public HandlerState state = HandlerState.STOWED;
  private TalonFX roller;
  private TalonFX pivot;
  private TalonFX indexer;
  private TalonFX effector;
  private CANrange canRange;
  private double currAngle;
  private double targetAngle;
  private double stowAngle;
  private Supplier<Distance> rangeSupplier;

  private double timeStarted;
  private double time;

  public CoralHandler() {
    roller = new TalonFX(ROLLERID, "Team 3045");
    pivot = new TalonFX(PIVOTID, "Team 3045");
    indexer = new TalonFX(INDEXERID, "Team 3045");
    effector = new TalonFX(EFFECTORID, "Team 3045");
    canRange = new CANrange(CANRANGEID, "Team 3045");
    stowAngle = pivot.getRotorPosition().getValueAsDouble();
    currAngle = stowAngle;
    targetAngle = stowAngle;
    rangeSupplier = canRange.getDistance().asSupplier();
    timeStarted = Timer.getTimestamp();
    time = Timer.getTimestamp();
  }

  public void changeState(HandlerState State) {
    state = State;
  }

  private void goToPosition(double angle) {
    targetAngle = angle;
  }

  private void goToStowPos() {
    goToPosition(stowAngle);
  }

  private void goToIntakingPos() {
    goToPosition(stowAngle + ANGLEDIFF);
    System.out.println("\u001B[34m\"Going to pos\u001B[0m");
  }

  private boolean atTargetPos() {
    return Math.abs(currAngle - targetAngle) <= ANGLETOLERANCE;
  }

  public void intake() {
    timeStarted = Timer.getTimestamp();
    state = HandlerState.INTAKING;
  }

  public void outtake() {
    timeStarted = Timer.getTimestamp();
    state = HandlerState.OUTTAKING;
  }

  @Override
  public void periodic() {
    currAngle = pivot.getRotorPosition().getValueAsDouble();
    time = Timer.getTimestamp();
    // This method will be called once per scheduler run
    pivot.setNeutralMode(NeutralModeValue.Brake);
    switch (state) {
      case INTAKING:
        goToIntakingPos();
        if (atTargetPos()) {
          roller.set(ROLLERSPEED);
          indexer.set(INDEXERSPEED);
          effector.set(EFFECTORSPEED);
          System.out.println("\u001B[34m\"Intaking\u001B[0m");
        }
        if ((time - timeStarted) >= INTAKELENGTHSECONDS) {
          state = HandlerState.IDLE;
        }
        if (rangeSupplier.get().lte(Inches.of(DEFAULTCANRANGEDIST))) {
          state = HandlerState.IDLE;
        }
        break;
      case OUTTAKING:
        goToIntakingPos();
        if (atTargetPos()) {
          roller.set(-ROLLERSPEED);
          indexer.set(-INDEXERSPEED);
          effector.set(-EFFECTORSPEED);
        }
        if ((time - timeStarted) >= INTAKELENGTHSECONDS) {
          state = HandlerState.IDLE;
        }
        if (rangeSupplier.get().lte(Inches.of(DEFAULTCANRANGEDIST))) {
          state = HandlerState.IDLE;
        }
        break;
      case STOWED:
        roller.set(0);
        indexer.set(0);
        effector.set(0);
        break;
      case IDLE:
        roller.set(0);
        indexer.set(0);
        effector.set(0);
        goToStowPos();
        if (atTargetPos()) {
          state = HandlerState.STOWED;
        }
        break;
    }
  }
}
