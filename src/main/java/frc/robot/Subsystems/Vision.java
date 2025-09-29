// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.LimelightCamera;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  ArrayList<LimelightCamera> cameras = new ArrayList<>();
  public Vision(int idx[]) {
    for (int i : idx) {
      cameras.add(new LimelightCamera(i));
    }
  }

  @Override
  public void periodic() {
    for (LimelightCamera cam : cameras) {
      cam.gather();
    }
  }
}
