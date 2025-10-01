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
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.generated.TunerConstants;
import frc.robot.vision.VisionConstants;
import frc.robot.Factories.AutoScoreAlgaeFactory;
import frc.robot.Factories.AutoScoreCoralFactory;
import frc.robot.Subsystems.AlgaeIntake;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.CoralHandler;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Vision;
import frc.robot.commons.GremlinLogger;
import frc.robot.commons.GremlinPS4Controller;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final GremlinPS4Controller joystick = new GremlinPS4Controller(0);
  private final CommandGenericHID buttonBoard = new CommandGenericHID(1);
  public static int poleHeight;
  
  // public final Intake intake = new Intake();

  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final static EndEffector effector = new EndEffector();
  public final static AlgaeIntake algaeintake = new AlgaeIntake(effector);
  public final static CoralHandler intake = new CoralHandler(effector);
  public final static Elevator elevator = new Elevator(algaeintake);
  public final static Climber climber = new Climber();
  public final static Vision vision = new Vision(VisionConstants.cameraIndeces);
  public final static AutoScoreCoralFactory scoreFactory = new AutoScoreCoralFactory();
  public final static AutoScoreAlgaeFactory algaeFactory = new AutoScoreAlgaeFactory();
  
  public static Pose3d[] componentPoses = new Pose3d[8];

  public void ConfigButtonBoard() {
    buttonBoard.button(1).onTrue(Commands.runOnce(() -> poleHeight = 3));
    buttonBoard.button(2).onTrue(Commands.runOnce(() -> poleHeight = 2));
    buttonBoard.button(3).onTrue(Commands.runOnce(() -> poleHeight = 1));
    buttonBoard.button(7).onTrue(Commands.runOnce(() -> poleHeight = 3));
    buttonBoard.button(8).onTrue(Commands.runOnce(() -> poleHeight = 2));
    buttonBoard.button(9).onTrue(Commands.runOnce(() -> poleHeight = 1));
  }

  public Command IntakeCoral() {
    return effector.Stow().andThen(elevator.Stow()).onlyWhile(effector.AtTargetAngle()).andThen(intake.Intake());
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
    joystick.R2().onTrue(IntakeCoral());
    joystick.R3().onTrue(intake.Outtake());
    joystick.share().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    joystick.R1().onTrue(Commands.runOnce(() -> scoreFactory.isRight = true).andThen(scoreFactory.fullAutoscore()));
    joystick.L1().onTrue(Commands.runOnce(() -> scoreFactory.isRight = false).andThen(scoreFactory.fullAutoscore()));
    joystick.L2().onTrue(algaeFactory.fullGrab());
    joystick.square().onTrue(elevator.GoToHeight(1).andThen(effector.GoToAngleDegrees(20)));
    joystick.triangle().OnPressTwice(climber.MoveOut(), climber.Climb());
    joystick.cross().onTrue(effector.Stow().andThen(elevator.Stow()));

    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
          drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );
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
  //   NamedCommands.registerCommand("intake", 
  //     intake.goToAngleDegrees(IntakeConstants.minAngle)
  //       .andThen(intake.applyDownwardCurrentFront()) // could be upward who knows
  //       //.andThen(Commands.waitUntil(nextMechanism.hasCoral?))
  //       .andThen(intake.stow()).alongWith(intake.zeroCurrentFront())
  //     );
  }
}
