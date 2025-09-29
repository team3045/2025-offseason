package frc.robot.Factories;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helpers;
import frc.robot.RobotContainer;
import frc.robot.Commands.DriveToPose;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.EndEffector;

import static frc.robot.Constants.AutoScoreConstants.*;

public class AutoScoreCoralFactory {
    private static CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
    private static Elevator elevator = RobotContainer.elevator;
    private static EndEffector effector = RobotContainer.effector;
    public boolean isRight = false;

    private Translation2d midpoint(Translation2d a, Translation2d b) {
        return new Translation2d(a.getX() + (b.getX() - a.getX())/2, a.getY() + (b.getY() - a.getY())/2);
    }

    private int getScorePole() {
        int scoreMidpoint = 0;
        double dist = 10000;
        int scorePole = 0;
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            for (int i = 0; i < 6; i ++) {
                double newDist = drivetrain.getState().Pose.getTranslation().getDistance(midpoint(POLEPOSESBLUE[2 * i].getTranslation(), POLEPOSESBLUE[2 * i + 1].getTranslation()));
                if (newDist < dist) {
                    dist = newDist;
                    scoreMidpoint = i;
                }
            }
            if (isRight) {
                scorePole = scoreMidpoint * 2 + 1;
            } else {
                scorePole = scoreMidpoint * 2;
            }
        } else {
            for (int i = 0; i < 6; i ++) {
                double newDist = drivetrain.getState().Pose.getTranslation().getDistance(midpoint(POLEPOSESRED[2 * i].getTranslation(), POLEPOSESRED[2 * i + 1].getTranslation()));
                if (newDist < dist) {
                    dist = newDist;
                    scoreMidpoint = i;
                }
            }
            if (isRight) {
                scorePole = scoreMidpoint * 2 + 1;
            } else {
                scorePole = scoreMidpoint * 2;
            }
        }
        return scorePole;
    }

    public Command goToElevatorHeight() {
        return elevator.GoToHeight(SCORE_HEIGHTS[Helpers.clamp(RobotContainer.poleHeight, 0, 3)]);
    }

    public Command goToEffectorAngle() {
        return effector.GoToAngleDegrees(SCORE_ANGLES[Helpers.clamp(RobotContainer.poleHeight, 0, 3)]);
    }

    public Command ejectCoral() {
        return effector.Outtake().until(() -> !effector.HasCoral().getAsBoolean());
    }

    public Command goToScorePos(int poleNum) {
        if (DriverStation.getAlliance().get() == Alliance.Blue)
            return new DriveToPose(drivetrain, () -> drivetrain.getState().Pose, () -> POLEPOSESBLUE[poleNum]);
        else
            return new DriveToPose(drivetrain, () -> drivetrain.getState().Pose, () -> POLEPOSESRED[poleNum]);
    }

    public Command fullAutoscore() {
        return goToScorePos(getScorePole()).alongWith(goToElevatorHeight()).andThen(goToEffectorAngle()).andThen(ejectCoral()).andThen(drivetrain.DriveBack()).andThen(effector.Stop()).andThen(effector.Stow()).andThen(elevator.Stow());
    }
}
