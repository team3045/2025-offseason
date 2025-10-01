package frc.robot.Constants;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class AutoScoreConstants {
    //Each pole in order
    public static Pose2d[] POLEPOSESRED = {
        new Pose2d(new Translation2d(3.18, 4.18), new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(new Translation2d(3.17, 3.850), new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(new Translation2d(3.69, 2.91), new Rotation2d(Units.degreesToRadians(60))),
        new Pose2d(new Translation2d(3.97, 2.76), new Rotation2d(Units.degreesToRadians(60))),
        new Pose2d(new Translation2d(5.02, 2.76), new Rotation2d(Units.degreesToRadians(120))),
        new Pose2d(new Translation2d(5.32, 2.94), new Rotation2d(Units.degreesToRadians(120))),
        new Pose2d(new Translation2d(5.85, 3.8), new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(new Translation2d(5.85, 4.21), new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(new Translation2d(5.30, 5.1), new Rotation2d(Units.degreesToRadians(240))),
        new Pose2d(new Translation2d(5.011, 5.26), new Rotation2d(Units.degreesToRadians(240))),
        new Pose2d(new Translation2d(3.97, 5.26), new Rotation2d(Units.degreesToRadians(300))),
        new Pose2d(new Translation2d(3.66, 5.08), new Rotation2d(Units.degreesToRadians(300)))
    };
    public static Pose2d[] POLEPOSESBLUE = {
        FlippingUtil.flipFieldPose(POLEPOSESRED[0]),
        FlippingUtil.flipFieldPose(POLEPOSESRED[1]),
        FlippingUtil.flipFieldPose(POLEPOSESRED[2]),
        FlippingUtil.flipFieldPose(POLEPOSESRED[3]),
        FlippingUtil.flipFieldPose(POLEPOSESRED[4]),
        FlippingUtil.flipFieldPose(POLEPOSESRED[5]),
        FlippingUtil.flipFieldPose(POLEPOSESRED[6]),
        FlippingUtil.flipFieldPose(POLEPOSESRED[7]),
        FlippingUtil.flipFieldPose(POLEPOSESRED[8]),
        FlippingUtil.flipFieldPose(POLEPOSESRED[9]),
        FlippingUtil.flipFieldPose(POLEPOSESRED[10]),
        FlippingUtil.flipFieldPose(POLEPOSESRED[11])
    };

    public static Pose2d[] ALGAEPOSESRED = {
        new Pose2d(new Translation2d(3.18, 4.0), new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(new Translation2d(3.85, 2.82), new Rotation2d(Units.degreesToRadians(60))),
        new Pose2d(new Translation2d(5.15, 2.85), new Rotation2d(Units.degreesToRadians(120))),
        new Pose2d(new Translation2d(5.85, 4.0), new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(new Translation2d(5.15, 5.18), new Rotation2d(Units.degreesToRadians(240))),
        new Pose2d(new Translation2d(3.18, 5.16), new Rotation2d(Units.degreesToRadians(300)))
    };

    public static Pose2d[] ALGAEPOSESBLUE = {
        FlippingUtil.flipFieldPose(ALGAEPOSESRED[0]),
        FlippingUtil.flipFieldPose(ALGAEPOSESRED[1]),
        FlippingUtil.flipFieldPose(ALGAEPOSESRED[2]),
        FlippingUtil.flipFieldPose(ALGAEPOSESRED[3]),
        FlippingUtil.flipFieldPose(ALGAEPOSESRED[4]),
        FlippingUtil.flipFieldPose(ALGAEPOSESRED[5])
    };

    public static Pose2d BARGEPOSEBLUE = new Pose2d(new Translation2d(), new Rotation2d());
    public static Pose2d BARGEPOSERED = new Pose2d(new Translation2d(), new Rotation2d());
    public static double BARGEELEVATORHEIGHT = 2;
    public static double BARGEENDEFFECTORANGLE = 0.5;

    public static double[] SCORE_HEIGHTS = {
        0.5,
        0.8,
        1.2,
        1.6
    };

    public static double[] ALGAEGRABHEIGHTS = {
        0.6,
        0.8
    };

    public static double ALGAEGRABANGLE = -40;

    public static double[] SCORE_ANGLES = {
        -45,
        -45,
        -45,
        0
    };

    public static Translation2d[] REEF_CENTERS = {
        new Translation2d(10, 10),
        new Translation2d(-10, 10)
    };

    public static double BACKUP_DIST = 1;
    public static double FRONT_DIST = 1;
}
