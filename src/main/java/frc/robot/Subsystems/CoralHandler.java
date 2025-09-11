// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Helpers.HandlerState;

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
  private Supplier<Distance> rangeSupplier;
  private HandlerState prevState;

  private double timeStarted;
  private double time;

  public CoralHandler() {
    roller = new TalonFX(ROLLERID, "Team 3045");
    pivot = new TalonFX(PIVOTID, "Team 3045");
    indexer = new TalonFX(INDEXERID, "Team 3045");
    effector = new TalonFX(EFFECTORID, "Team 3045");
    canRange = new CANrange(CANRANGEID, "Team 3045");
    rangeSupplier = canRange.getDistance().asSupplier();
    timeStarted = Timer.getTimestamp();
    time = Timer.getTimestamp();
  }

  public void intake() {
    timeStarted = Timer.getTimestamp();
    prevState = HandlerState.INTAKING;
    state = HandlerState.MOVINGDOWN;
  }

  public void outtake() {
    timeStarted = Timer.getTimestamp();
    prevState = HandlerState.OUTTAKING;
    state = HandlerState.MOVINGDOWN;
  }

  @Override
  public void periodic() {
    time = Timer.getTimestamp();
    SmartDashboard.putNumber("Range", rangeSupplier.get().baseUnitMagnitude());
    SmartDashboard.putNumber("PivotCurrent", Math.abs(pivot.getTorqueCurrent().getValueAsDouble()));
    SmartDashboard.putNumber("PivotVelocity", Math.abs(pivot.getVelocity().getValueAsDouble()));
    SmartDashboard.putString("IntakeState", state.toString());
    // This method will be called once per scheduler run
    pivot.setNeutralMode(NeutralModeValue.Brake);
    switch (state) {
      case MOVINGDOWN:
        pivot.set(-PIVOTSPEED);
        if (Math.abs(pivot.getTorqueCurrent().getValueAsDouble()) > 50 && Math.abs(pivot.getVelocity().getValueAsDouble()) < 5) {
          state = prevState;
          pivot.set(0.03);
        }
        break;
      case MOVINGUP:
        pivot.set(PIVOTSPEED);
        if (Math.abs(pivot.getTorqueCurrent().getValueAsDouble()) > 50 && Math.abs(pivot.getVelocity().getValueAsDouble()) < 5) {
          state = prevState;
          pivot.set(0.03);
        }
        break;
      case INTAKING:
        roller.set(ROLLERSPEED);
        indexer.set(INDEXERSPEED);
        effector.set(EFFECTORSPEED);
        if ((time - timeStarted) >= INTAKELENGTHSECONDS) {
          state = HandlerState.IDLE;
        }
        if (rangeSupplier.get().baseUnitMagnitude() <= DEFAULTCANRANGEDIST) {
          state = HandlerState.IDLE;
        }
        break;
      case OUTTAKING:
        roller.set(-ROLLERSPEED);
        indexer.set(-INDEXERSPEED);
        effector.set(-EFFECTORSPEED);
        if ((time - timeStarted) >= INTAKELENGTHSECONDS) {
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
        prevState = HandlerState.STOWED;
        state = HandlerState.MOVINGUP;
        break;
    }
  }
}
