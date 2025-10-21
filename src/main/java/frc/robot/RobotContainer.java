// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLogOptions;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.vision.VisionConstants;
import frc.robot.Commands.Stow;
import frc.robot.Factories.AutoScoreAlgaeFactory;
import frc.robot.Factories.AutoScoreCoralFactory;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.CoralHandler;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Vision;
import frc.robot.commons.GremlinLogger;
import frc.robot.commons.GremlinPS4Controller;
import frc.robot.Commands.*;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final GremlinPS4Controller joystick = new GremlinPS4Controller(0);
  
  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final static EndEffector effector = new EndEffector();
  public final static CoralHandler intake = new CoralHandler(effector);
  public final static Elevator elevator = new Elevator();
  public final static Climber climber = new Climber();
  public final static Vision vision = new Vision(VisionConstants.cameraIndeces);
  public final static AutoScoreCoralFactory scoreFactory = new AutoScoreCoralFactory();
  public final static AutoScoreAlgaeFactory algaeFactory = new AutoScoreAlgaeFactory();
  
  public static Pose3d[] componentPoses = new Pose3d[8];

  public Command IntakeCoral() {
    return new Stow().andThen(intake.Intake());
  }

  public Command OuttakeCoral() {
    return new Stow().andThen(new GoToHeightAndAngle(0.7, 0.45)).andThen(intake.Outtake()).andThen(Commands.waitSeconds(1)).andThen(new Stow());
  }

  public RobotContainer() {
    GremlinLogger.setOptions(new DogLogOptions()
      .withNtPublish(false)
      .withCaptureNt(true)
      .withCaptureConsole(true)
      .withLogExtras(false));
    
    // auto stuff goes here when made

    configureBindings();
  }

  private void configureBindings() {
    joystick.R2().onTrue(IntakeCoral()); //Intake
    joystick.L2().onTrue(OuttakeCoral()); //Outtake
    joystick.share().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); //Zero
    joystick.R1().onTrue(Commands.runOnce(() -> scoreFactory.isLeft = false)); //Right
    joystick.L1().onTrue(Commands.runOnce(() -> scoreFactory.isLeft = true)); //Left
    joystick.povUp().onTrue(Commands.runOnce(() -> scoreFactory.scoreHeight = 3)); //L4
    joystick.povRight().onTrue(Commands.runOnce(() -> scoreFactory.scoreHeight = 2)); //L3
    joystick.povDown().onTrue(Commands.runOnce(() -> scoreFactory.scoreHeight = 1)); //L2
    joystick.povLeft().onTrue(Commands.runOnce(() -> scoreFactory.scoreHeight = 0)); //L1
    joystick.R3().onTrue(scoreFactory.fullAutoscore()); //Score

    joystick.cross().onTrue(new GoToHeightAndAngle(0, 0)); //Stow (moves to safe height and then stows)
    joystick.circle().onTrue(scoreFactory.ejectCoral()); //Eject coral
    joystick.triangle().OnPressTwice(climber.MoveOut(), climber.Climb()); //Climb (one press move out, second press move in)

    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
          drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );

    // ConfigButtonBoard();
    registerPathPlannerCommands();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void registerPathPlannerCommands(){
    NamedCommands.registerCommand("IntakeCoral", IntakeCoral());
    NamedCommands.registerCommand("BargeScore", algaeFactory.fullBarge());
    NamedCommands.registerCommand("8ScoreL4", scoreFactory.fullAutoscore(8, 3));
    NamedCommands.registerCommand("7ScoreL4", scoreFactory.fullAutoscore(7, 3));
    NamedCommands.registerCommand("6ScoreL4", scoreFactory.fullAutoscore(6, 3));
    NamedCommands.registerCommand("5ScoreL4", scoreFactory.fullAutoscore(5, 3));
    NamedCommands.registerCommand("4ScoreL4", scoreFactory.fullAutoscore(4, 3));
    NamedCommands.registerCommand("3ScoreL4", scoreFactory.fullAutoscore(3, 3));
    NamedCommands.registerCommand("2GrabAlgae", algaeFactory.fullGrab(2));
    NamedCommands.registerCommand("3GrabAlgae", algaeFactory.fullGrab(3));
    NamedCommands.registerCommand("4GrabAlgae", algaeFactory.fullGrab(4));
  }
}
