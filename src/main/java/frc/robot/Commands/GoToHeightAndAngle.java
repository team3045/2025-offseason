// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToHeightAndAngle extends Command {
  /** Creates a new GoToHeightAndAngle. */
  private Elevator elevator = RobotContainer.elevator;
  private EndEffector endEffector = RobotContainer.effector;
  private double targetHeight;
  private double targetRotations;
  private boolean isAtSafeHeight;
  private boolean rotated;
  public GoToHeightAndAngle(double TargetHeight, double TargetRotations) {
    targetHeight = TargetHeight;
    targetRotations = TargetRotations;
    isAtSafeHeight = false;
    rotated = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.GoToHeight(ElevatorConstants.SAFETURNHEIGHT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!isAtSafeHeight) {
      isAtSafeHeight = elevator.atTargetHeight();
      if (!isAtSafeHeight) {
        return;
      }
    }
    endEffector.GoToRot(targetRotations);
    if (!rotated) {
      rotated = endEffector.atTargetAngle();
      if (!rotated) {
        return;
      }
    }
    elevator.goToHeight(targetHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.atTargetHeight() && endEffector.atTargetAngle();
  }
}
