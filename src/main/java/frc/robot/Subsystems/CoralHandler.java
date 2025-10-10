// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private HandlerState prevState;
  private EndEffector effector;
  private SoftwareLimitSwitchConfigs config;

  private double timeStarted;
  private double time;
  private double topZero;

  public CoralHandler(EndEffector Effector) {
    roller = new TalonFX(ROLLERID, "Team 3045");
    pivot = new TalonFX(PIVOTID, "Team 3045");
    indexer = new TalonFX(INDEXERID, "Team 3045");
    config = new SoftwareLimitSwitchConfigs();
    config.ForwardSoftLimitEnable = false;
    config.ReverseSoftLimitEnable = false;
    pivot.getConfigurator().apply(config);
    effector = Effector;
    timeStarted = Timer.getTimestamp();
    time = Timer.getTimestamp();
    topZero = 0;
  }

  public void intake() {
    timeStarted = Timer.getTimestamp();
    prevState = HandlerState.INTAKING;
    state = HandlerState.MOVINGDOWN;
  }

  public void clearWay() {
    timeStarted = Timer.getTimestamp();
    prevState = HandlerState.CLEARINGWAY;
    state = HandlerState.MOVINGUP;
  }

  public void outtake() {
    timeStarted = Timer.getTimestamp();
    prevState = HandlerState.OUTTAKING;
    state = HandlerState.MOVINGDOWN;
  }

  public void stow() {
    timeStarted = Timer.getTimestamp();
    prevState = HandlerState.STOWED;
    state = HandlerState.MOVINGUP;
  }

  public Command Intake() {
    return this.runOnce(() -> intake());
  }

  public Command Outtake() {
    return this.runOnce(() -> outtake());
  }

  public Command Stow() {
    return this.runOnce(() -> stow());
  }

  public Command ClearWay() {
    return this.runOnce(() -> clearWay());
  }

  @Override
  public void periodic() {
    time = Timer.getTimestamp();
    SmartDashboard.putString("CoralHandler/IntakeState", state.toString());
    SmartDashboard.putBoolean("CoralHandler/HasCoral", effector.HasCoral().getAsBoolean());
    // This method will be called once per scheduler run
    pivot.setNeutralMode(NeutralModeValue.Brake);
    switch (state) {
      case MOVINGDOWN:
        pivot.set(-PIVOTSPEED);
        if (Math.abs(pivot.getTorqueCurrent().getValueAsDouble()) > 40 && Math.abs(pivot.getVelocity().getValueAsDouble()) < 0.1) {
          state = prevState;
          pivot.set(0.03);
        }
        break;
      case MOVINGUP:
        pivot.set(PIVOTSPEED);
        if (Math.abs(pivot.getTorqueCurrent().getValueAsDouble()) > 40 && Math.abs(pivot.getVelocity().getValueAsDouble()) < 0.1) {
          state = prevState;
          pivot.set(0.03);
          topZero = pivot.getRotorPosition().getValueAsDouble();
        }
        break;
      case INTAKING:
        roller.set(ROLLERSPEED);
        indexer.set(INDEXERSPEED);
        effector.setRunning(true);
        effector.setDir(1);
        if ((time - timeStarted) >= INTAKELENGTHSECONDS) {
          state = HandlerState.IDLE;
        }
        if (effector.HasCoral().getAsBoolean()) {
          state = HandlerState.IDLE;
        }
        break;
      case OUTTAKING:
        roller.set(-ROLLERSPEED);
        indexer.set(-INDEXERSPEED);
        effector.setRunning(true);
        effector.setDir(-1);
        if ((time - timeStarted) >= INTAKELENGTHSECONDS) {
          state = HandlerState.IDLE;
        }
        break;
      case STOWED:
        roller.set(0);
        indexer.set(0);
        effector.setRunning(false);
        break;
      case IDLE:
        roller.set(0);
        indexer.set(0);
        effector.setRunning(false);
        prevState = HandlerState.STOWED;
        state = HandlerState.MOVINGUP;
        break;
      case CLEARINGWAY:
        double pos = pivot.getRotorPosition().getValueAsDouble();
        double dist = (topZero - pos) - CLEARWAYAMOUNT;
        SmartDashboard.putNumber("CoralHandler/dist", dist);
        pivot.set(dist/10);
        break;
    }
  }
}
