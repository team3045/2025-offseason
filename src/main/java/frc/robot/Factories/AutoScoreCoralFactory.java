package frc.robot.Factories;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Commands.DriveToPose;
import frc.robot.Commands.GoToHeightAndAngle;
import frc.robot.Commands.Stow;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.EndEffector;

import static frc.robot.Constants.AutoScoreConstants.*;

import java.util.function.Supplier;

public class AutoScoreCoralFactory {
    private static CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
    private static EndEffector effector = RobotContainer.effector;

    public int getScorePole(boolean isLeft) {
        int scorePole = 0;
        double minDist = 1000000;
        for (int i = 0; i < 6; i ++) {
            Rotation2d rot;
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                rot = POLEPOSESBLUE[(2 * i)].getRotation();
            } else {
                rot = POLEPOSESRED[(2 * i)].getRotation();
            }
            double dist = Math.abs(drivetrain.getState().Pose.getRotation().getDegrees() - rot.getDegrees());
            if (dist < minDist) {
                minDist = dist;
                scorePole = i * 2;
            }
        }
        if (isLeft) scorePole ++;
        SmartDashboard.putNumber("ScorePos/PoleNum", scorePole);
        return scorePole;
    }

    public Command goToElevatorHeightAndEffectorAngle(int height) {
        return new GoToHeightAndAngle(SCORE_HEIGHTS[height], SCORE_ANGLES[height]);
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

    public Command fullAutoscore(boolean isLeft, int height) {
        return goToScorePos(getScorePole(isLeft)).alongWith(goToElevatorHeightAndEffectorAngle(height)).andThen(drivetrain.DriveFoward()).andThen(Commands.waitSeconds(1)).andThen(ejectCoral()).andThen(drivetrain.DriveBack()).andThen(effector.Stop()).andThen(new Stow());
    }

    public Command fullAutoscore(int poleNum, int height) {
        return goToScorePos(poleNum).alongWith(goToElevatorHeightAndEffectorAngle(height)).andThen(drivetrain.DriveFoward()).andThen(Commands.waitSeconds(1)).andThen(ejectCoral()).andThen(drivetrain.DriveBack()).andThen(effector.Stop()).andThen(new Stow());
    }
}
