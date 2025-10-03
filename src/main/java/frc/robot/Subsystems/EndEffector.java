// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static frc.robot.Constants.EndEffectorConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
  private double currRot;
  private TalonFX effectorMotor;
  private TalonFX effectorTilterMotor;
  private CANcoder encoderTop;
  public double targetRot;
  private boolean isEffectorRunning;
  private int dirMult;
  private double zero;
  private CANrange coralRange;
  private CANrange algaeRange;
  private Supplier<Distance> coralSupplier;
  private Supplier<Distance> algaeSupplier;
  private int turnDir;

  public EndEffector() {
    effectorMotor = new TalonFX(EFFECTORID, "Team 3045");
    effectorTilterMotor = new TalonFX(EFFECTORTILTERID, "Team 3045");
    encoderTop = new CANcoder(ENCODERTOPID, "Team 3045");
    coralRange = new CANrange(CORALRANGEID, "Team 3045");
    algaeRange = new CANrange(ALGAERANGEID, "Team 3045");
    coralSupplier = coralRange.getDistance().asSupplier();
    algaeSupplier = algaeRange.getDistance().asSupplier();
    zero = encoderTop.getAbsolutePosition().getValueAsDouble();
    currRot = zero;
    targetRot = 0;
    isEffectorRunning = false;
    dirMult = 1;
  }

  public void stow() {
    turnDir = STOWDIR;
    goToRot(STOWANGLE);
  }

  public Command Stow() {
    return this.runOnce(() -> stow());
  }

  private void goToRot(double TargetRot) {
    targetRot = TargetRot;
  }

  public Command GoToRot(double targetRot) {
    return this.runOnce(() -> goToRotations(targetRot));
  }

  public void goToRotations(double targetRot) {
    turnDir = -STOWDIR;
    goToRot(targetRot);
  }

  public void goToAngleDegrees(double angle) {
    turnDir = -STOWDIR;
    goToRot(Units.degreesToRotations(angle));
  }

  public Command GoToAngleDegrees(double angle) {
    return this.runOnce(() -> goToAngleDegrees(angle));
  }

  public boolean atTargetAngle() {
    return Math.abs(targetRot - currRot) < ANGLETOLERANCE;
  }

  public BooleanSupplier AtTargetAngle() {
    return () -> atTargetAngle();
  }

  public void setRunning(boolean state) {
    isEffectorRunning = state;
  }

  public void setDir(int dir) {
    dirMult = dir;
  }

  public Command Outtake() {
    return this.runOnce(() -> {setDir(-1); setRunning(true);});
  }

  public Command Intake() {
    return this.runOnce(() -> {setDir(1); setRunning(true);});
  }

  public Command Stop() {
    return this.runOnce(() -> {setRunning(false);});
  }

  public BooleanSupplier HasCoral() {
    return () -> coralSupplier.get().baseUnitMagnitude() <= DEFAULTCORALDIST;
  }

  public BooleanSupplier HasAlgae() {
    return () -> algaeSupplier.get().baseUnitMagnitude() <= DEFAULTALGAEDIST;
  }

  @Override
  public void periodic() {
    currRot = encoderTop.getAbsolutePosition().getValueAsDouble() - zero;
    double rotDiff = 0;
    rotDiff = Math.abs(currRot - targetRot);

    double speed = (rotDiff/SPEEDDOWN) * turnDir;
    SmartDashboard.putNumber("EndEffector/CurrRot", currRot);
    SmartDashboard.putNumber("EndEffector/Zero", zero);
    SmartDashboard.putNumber("EndEffector/RotDiff", speed);
    SmartDashboard.putNumber("EndEffector/TargetRot", targetRot);
    
    if (Math.abs(rotDiff) > ANGLETOLERANCE) {
      effectorTilterMotor.set(speed);
    } else {
      effectorTilterMotor.set(0);
    }
    if (isEffectorRunning) {
      effectorMotor.set(EFFECTORSPEED * dirMult);
    } else {
      effectorMotor.set(EFFECTORHOLDSPEED);
    }
  }
}
