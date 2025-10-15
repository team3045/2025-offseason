package frc.robot.Constants;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class AutoScoreConstants {
    //Each pole in order
    public static Pose2d[] POLEPOSESBLUE = {
        new Pose2d(new Translation2d(2.73, 3.86), new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(new Translation2d(2.73, 4.23), new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(new Translation2d(3.52, 5.49), new Rotation2d(Units.degreesToRadians(-60))),
        new Pose2d(new Translation2d(3.82, 5.70), new Rotation2d(Units.degreesToRadians(-60))),
        new Pose2d(new Translation2d(5.33, 5.59), new Rotation2d(Units.degreesToRadians(-120))),
        new Pose2d(new Translation2d(5.69, 5.31), new Rotation2d(Units.degreesToRadians(-120))),
        new Pose2d(new Translation2d(6.34, 4.05), new Rotation2d(Units.degreesToRadians(-180))),
        new Pose2d(new Translation2d(6.34, 3.75), new Rotation2d(Units.degreesToRadians(-180))),
        new Pose2d(new Translation2d(5.37, 2.47), new Rotation2d(Units.degreesToRadians(-240))),
        new Pose2d(new Translation2d(5.12, 2.23), new Rotation2d(Units.degreesToRadians(-240))),
        new Pose2d(new Translation2d(4.04, 2.39), new Rotation2d(Units.degreesToRadians(-300))),
        new Pose2d(new Translation2d(3.35, 2.66), new Rotation2d(Units.degreesToRadians(-300)))
    };
    public static Pose2d[] POLEPOSESRED = {
        FlippingUtil.flipFieldPose(POLEPOSESBLUE[0]),
        FlippingUtil.flipFieldPose(POLEPOSESBLUE[1]),
        FlippingUtil.flipFieldPose(POLEPOSESBLUE[2]),
        FlippingUtil.flipFieldPose(POLEPOSESBLUE[3]),
        FlippingUtil.flipFieldPose(POLEPOSESBLUE[4]),
        FlippingUtil.flipFieldPose(POLEPOSESBLUE[5]),
        FlippingUtil.flipFieldPose(POLEPOSESBLUE[6]),
        FlippingUtil.flipFieldPose(POLEPOSESBLUE[7]),
        FlippingUtil.flipFieldPose(POLEPOSESBLUE[8]),
        FlippingUtil.flipFieldPose(POLEPOSESBLUE[9]),
        FlippingUtil.flipFieldPose(POLEPOSESBLUE[10]),
        FlippingUtil.flipFieldPose(POLEPOSESBLUE[11])
    };

    public static Pose2d[] ALGAEPOSESBLUE = {
        new Pose2d(new Translation2d(3.18, 4.0), new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(new Translation2d(3.85, 2.82), new Rotation2d(Units.degreesToRadians(60))),
        new Pose2d(new Translation2d(5.15, 2.85), new Rotation2d(Units.degreesToRadians(120))),
        new Pose2d(new Translation2d(5.85, 4.0), new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(new Translation2d(5.15, 5.18), new Rotation2d(Units.degreesToRadians(240))),
        new Pose2d(new Translation2d(3.18, 5.16), new Rotation2d(Units.degreesToRadians(300)))
    };

    public static Pose2d[] ALGAEPOSESRED = {
        FlippingUtil.flipFieldPose(ALGAEPOSESBLUE[0]),
        FlippingUtil.flipFieldPose(ALGAEPOSESBLUE[1]),
        FlippingUtil.flipFieldPose(ALGAEPOSESBLUE[2]),
        FlippingUtil.flipFieldPose(ALGAEPOSESBLUE[3]),
        FlippingUtil.flipFieldPose(ALGAEPOSESBLUE[4]),
        FlippingUtil.flipFieldPose(ALGAEPOSESBLUE[5])
    };

    public static double BARGEELEVATORHEIGHT = 2;
    public static double BARGEENDEFFECTORANGLE = 0.5;

    public static double[] SCORE_HEIGHTS = {
        0,
        0.28,
        0.7,
        1.45
    };

    public static double[] ALGAEGRABHEIGHTS = {
        0.5,
        0.7
    };

    public static double ALGAEGRABANGLE = 0.9;

    public static double[] SCORE_ANGLES = {
        0.55,
        0.55,
        0.55,
        0.655
    };

    public static Translation2d[] REEF_CENTERS = {
        new Translation2d(4, 4),
        new Translation2d(13.69, 4)
    };
}
