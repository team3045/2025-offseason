// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static frc.robot.Constants.EndEffectorConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
  private double currRot;
  private TalonFX effectorMotor;
  private TalonFX effectorTilterMotor;
  private CANcoder encoderBottom;
  private CANcoder encoderTop;
  private double targetRot;
  private boolean isEffectorRunning;
  private int dirMult;
  private double zero;

  public EndEffector() {
    effectorMotor = new TalonFX(EFFECTORID, "Team 3045");
    effectorTilterMotor = new TalonFX(EFFECTORTILTERID, "Team 3045");
    encoderBottom = new CANcoder(ENCODERBOTTOMID, "Team 3045");
    encoderTop = new CANcoder(ENCODERTOPID, "Team 3045");
    double numRotations = Math.floor((encoderTop.getAbsolutePosition().getValueAsDouble() - encoderBottom.getAbsolutePosition().getValueAsDouble())/ENCODERROTDIFFPERFULLROT);
    zero = encoderTop.getAbsolutePosition().getValueAsDouble();
    currRot = zero;
    targetRot = 0;
    isEffectorRunning = false;
    dirMult = 1;
  }

  public void stow() {
    goToRot(STOWROT);
  }

  public void goToRot(double TargetRot) {
    targetRot = TargetRot;
  }

  public boolean atTargetAngle() {
    return Math.abs(targetRot - currRot) < ANGLETOLERANCE;
  }

  public void setRunning(boolean state) {
    isEffectorRunning = state;
  }

  public void setDir(int dir) {
    dirMult = dir;
  }

  @Override
  public void periodic() {
    currRot = encoderTop.getAbsolutePosition().getValueAsDouble() - zero;
    double rotDiff = currRot - (targetRot + 0.04);
    double rotDiffShifted = currRot + 1 - targetRot;
    if (Math.abs(rotDiffShifted) < Math.abs(rotDiff)) {
      rotDiff = rotDiffShifted;
    }
    double speed = -rotDiff/SPEEDDOWN;
    SmartDashboard.putNumber("EndEffector/CurrRot", currRot);
    SmartDashboard.putNumber("EndEffector/Zero", zero);
    SmartDashboard.putNumber("EndEffector/RotDiff", speed);
    SmartDashboard.putNumber("EndEffector/TargetRot", targetRot);
    
    if (Math.abs(rotDiff) > ANGLETOLERANCE) {
      effectorTilterMotor.set(speed);
    } else {
      // effectorTilterMotor.set(Math.signum(speed) * HOLDSPEED);
      effectorTilterMotor.set(0);
    }
    if (isEffectorRunning) {
      effectorMotor.set(EFFECTORSPEED * dirMult);
    } else {
      effectorMotor.set(EFFECTORHOLDSPEED);
    }
  }
}
