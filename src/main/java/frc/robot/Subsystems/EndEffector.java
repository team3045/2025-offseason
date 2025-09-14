// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static frc.robot.Constants.EndEffectorConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
  /** Creates a new EndEffector. */
  private double currRot;
  private TalonFX effectorMotor;
  private TalonFX effectorTilterMotor;
  private CANcoder encoderBottom;
  private CANcoder encoderTop;
  private double targetRot;
  private boolean isEffectorRunning;
  private int dirMult;

  public EndEffector() {
    effectorMotor = new TalonFX(EFFECTORID, "Team 3045");
    effectorTilterMotor = new TalonFX(EFFECTORTILTERID, "Team 3045");
    encoderBottom = new CANcoder(ENCODERBOTTOMID, "Team 3045");
    encoderTop = new CANcoder(ENCODERTOPID, "Team 3045");
    double numRotations = Math.floor((encoderTop.getAbsolutePosition().getValueAsDouble() - encoderBottom.getAbsolutePosition().getValueAsDouble())/ENCODERROTDIFFPERFULLROT);
    currRot = encoderTop.getAbsolutePosition().getValueAsDouble() + numRotations;
    targetRot = currRot;
    isEffectorRunning = false;
    dirMult = 1;
  }

  public void stow() {
    goToAngle(STOWROT);
  }

  public void goToAngle(double targetAngle) {
    targetRot = targetAngle;
  }

  public void setRunning(boolean state) {
    isEffectorRunning = state;
  }

  public void setDir(int dir) {
    dirMult = dir;
  }

  @Override
  public void periodic() {
    double rotDiff = currRot - targetRot;
    double speed = rotDiff/SPEEDDOWN;
    if (Math.abs(rotDiff) > ANGLETOLERANCE) {
      effectorTilterMotor.set(speed);
    } else {
      effectorTilterMotor.set(HOLDSPEED);
    }
    if (isEffectorRunning) {
      effectorMotor.set(EFFECTORSPEED * dirMult);
    } else {
      effectorMotor.set(EFFECTORHOLDSPEED);
    }
    // This method will be called once per scheduler run
  }
}
