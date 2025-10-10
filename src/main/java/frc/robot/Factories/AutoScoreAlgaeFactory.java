// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helpers;
import frc.robot.RobotContainer;
import frc.robot.Commands.DriveToPose;
import frc.robot.Commands.GoToHeightAndAngle;
import frc.robot.Commands.Stow;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.EndEffector;

import static frc.robot.Constants.AutoScoreConstants.*;

/** Add your docs here. */
public class AutoScoreAlgaeFactory {
    private static CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
    private static Elevator elevator = RobotContainer.elevator;
    private static EndEffector effector = RobotContainer.effector;
    private static int algaeNum = 1;

    private static int getAlgaeNum() {
        int algaeMid = 0;
        double dist = 10000;
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            for (int i = 0; i < 6; i ++) {
                double newDist = drivetrain.getState().Pose.getTranslation().getDistance(ALGAEPOSESBLUE[i].getTranslation());
                if (newDist < dist) {
                    dist = newDist;
                    algaeMid = i;
                }
            }
            return algaeMid;
        } else {
            for (int i = 0; i < 6; i ++) {
                double newDist = drivetrain.getState().Pose.getTranslation().getDistance(ALGAEPOSESRED[i].getTranslation());
                if (newDist < dist) {
                    dist = newDist;
                    algaeMid = i;
                }
            }
            return algaeMid;
        }
    }

    private static double getAlgaeGrabHeight() {
        if (DriverStation.getAlliance().get() == Alliance.Blue)
            return ALGAEGRABHEIGHTS[algaeNum % 2];
        else
            return ALGAEGRABHEIGHTS[(algaeNum + 1) % 2];
        
    }

    private static Command goToPose() {
        algaeNum = Helpers.clamp(algaeNum, 0, 5);
        if (DriverStation.getAlliance().get() == Alliance.Blue)
            return new DriveToPose(drivetrain, () -> drivetrain.getState(), () -> ALGAEPOSESBLUE[algaeNum]);
        else
            return new DriveToPose(drivetrain, () -> drivetrain.getState(), () -> ALGAEPOSESRED[algaeNum]);
    }
    
    public Command fullGrab() {
        algaeNum = getAlgaeNum();
        return goToPose().alongWith(new GoToHeightAndAngle(getAlgaeGrabHeight(), ALGAEGRABANGLE)).andThen(drivetrain.DriveFoward()).andThen(effector.Intake()).until(effector.HasAlgae()).withTimeout(1).andThen(effector.Stop()).andThen(drivetrain.DriveBack()).andThen(new Stow());
    }

    public Command fullGrab(int AlgaeNum) {
        algaeNum = AlgaeNum;
        return goToPose().alongWith(new GoToHeightAndAngle(getAlgaeGrabHeight(), ALGAEGRABANGLE)).andThen(drivetrain.DriveFoward()).andThen(effector.Intake()).until(effector.HasAlgae()).withTimeout(1).andThen(effector.Stop()).andThen(drivetrain.DriveBack()).andThen(new Stow());
    }

    public Command fullBarge() {
        return new GoToHeightAndAngle(BARGEELEVATORHEIGHT, BARGEENDEFFECTORANGLE).andThen(drivetrain.DriveBack()).andThen(effector.Outtake()).andThen(drivetrain.DriveFoward()).andThen(new Stow());
    }
}
