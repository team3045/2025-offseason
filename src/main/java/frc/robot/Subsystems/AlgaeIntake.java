// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static frc.robot.Constants.AlgaeIntakeConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  private TalonFX pivot;
  private TalonFX roller;
  private CANrange canRange;
  private Supplier<Distance> rangeSupplier;
  private EndEffector effector;
  private double startRot;
  private double targetRot;
  private boolean runRollers;

  public AlgaeIntake(EndEffector Effector) {
    roller = new TalonFX(ROLLERID, "Team 3045");
    pivot = new TalonFX(PIVOTID, "Team 3045");
    canRange = new CANrange(CANRANGEID, "Team 3045");
    effector = Effector;
    rangeSupplier = canRange.getDistance().asSupplier();
    startRot = pivot.getRotorPosition().getValueAsDouble();
    targetRot = 0;
    runRollers = false;
  }

  public void intake() {
    targetRot = TOTALROTATIONSFORINTAKE;
    runRollers = true;
  }

  public void outtake() {
    targetRot = TOTALROTATIONSFORINTAKE;
    runRollers = true;
  }

  public void moveDown() {
    targetRot = TOTALROTATIONSFORINTAKE;
    runRollers = false;
  }

  public void moveUp() {
    targetRot = 0;
    runRollers = false;
  }

  public void clearWay() {
    targetRot = TOTALROTATIONSFORCOLLISION;
    runRollers = false;
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

  public Command ClearWay() {
    return this.runOnce(() -> clearWay());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("AlgaeIntake/Range", rangeSupplier.get().baseUnitMagnitude());
    SmartDashboard.putNumber("AlgaeIntake/TargetRot", targetRot);
    SmartDashboard.putNumber("AlgaeIntake/CurrRot", pivot.getRotorPosition().getValueAsDouble() - startRot);
    // This method will be called once per scheduler run
    pivot.setNeutralMode(NeutralModeValue.Brake);
    double diff = (pivot.getRotorPosition().getValueAsDouble() - startRot) - targetRot;
    SmartDashboard.putNumber("AlgaeIntake/Diff", diff);
    double speed = diff/16;

    if (runRollers) {
      roller.set(ROLLERSPEED);
    } else {
      roller.set(0);
    }
    if (Math.abs(diff) > 0.1) {
      pivot.set(-Math.signum(speed) * Math.min(Math.abs(speed), 0.1));
    }
  }
}
