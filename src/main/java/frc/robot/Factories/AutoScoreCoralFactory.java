package frc.robot.Factories;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Helpers;
import frc.robot.RobotContainer;
import frc.robot.Commands.DriveToPose;
import frc.robot.Commands.GoToHeightAndAngle;
import frc.robot.Commands.Stow;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.EndEffector;

import static frc.robot.Constants.AutoScoreConstants.*;

public class AutoScoreCoralFactory {
    private static CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
    private static EndEffector effector = RobotContainer.effector;
    public boolean isLeft = false;
    public int scoreHeight = 3;

    private Translation2d midpoint(Translation2d a, Translation2d b) {
        return new Translation2d((b.getX() + a.getX())/2, (b.getY() + a.getY())/2);
    }

    private int getScorePole() {
        int scorePole = 0;
        double minDist = 1000000;
        for (int i = 0; i < 6; i ++) {
            Translation2d trans;
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                trans = midpoint(POLEPOSESBLUE[2 * i].getTranslation(), POLEPOSESBLUE[2 * i + 1].getTranslation());
            } else {
                trans = midpoint(POLEPOSESRED[2 * i].getTranslation(), POLEPOSESRED[2 * i + 1].getTranslation());
            }
            double dist = drivetrain.getState().Pose.getTranslation().getDistance(trans);
            if (dist < minDist) {
                minDist = dist;
                scorePole = i * 2;
            }
        }
        if (isLeft) scorePole ++;
        return scorePole;
    }

    public Command goToElevatorHeightAndEffectorAngle(int height) {
        return new GoToHeightAndAngle(SCORE_HEIGHTS[Helpers.clamp(height, 0, 3)], SCORE_ANGLES[Helpers.clamp(height, 0, 3)]);
    }

    public Command ejectCoral() {
        return effector.Outtake().until(() -> !effector.HasCoral().getAsBoolean());
    }

    public Command goToScorePos(int poleNum) {
        if (DriverStation.getAlliance().get() == Alliance.Blue)
            return new DriveToPose(drivetrain, () -> drivetrain.getState(), () -> POLEPOSESBLUE[poleNum]);
        else
            return new DriveToPose(drivetrain, () -> drivetrain.getState(), () -> POLEPOSESRED[poleNum]);
    }

    public Command fullAutoscore() {
        return goToScorePos(getScorePole()).alongWith(goToElevatorHeightAndEffectorAngle(scoreHeight)).andThen(drivetrain.DriveFoward()).andThen(Commands.waitSeconds(1)).andThen(ejectCoral()).andThen(drivetrain.DriveBack()).andThen(effector.Stop()).andThen(new Stow());
    }

    public Command fullAutoscore(int poleNum, int height) {
        return goToScorePos(poleNum).alongWith(goToElevatorHeightAndEffectorAngle(height)).andThen(drivetrain.DriveFoward()).andThen(Commands.waitSeconds(1)).andThen(ejectCoral()).andThen(drivetrain.DriveBack()).andThen(effector.Stop()).andThen(new Stow());
    }
}
