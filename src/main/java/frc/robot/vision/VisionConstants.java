// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class VisionConstants {
    public static final String[] cameraNames = new String[]{"limelight-left"};
    public static final Pose3d[] cameraPoses = new Pose3d[]{new Pose3d(
        new Translation3d(Units.inchesToMeters(5), Units.inchesToMeters(-16), Units.inchesToMeters(9)),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-12.0), Units.degreesToRadians(100.0)))};
    public static final int cameraIndeces[] = new int[]{0};
}
