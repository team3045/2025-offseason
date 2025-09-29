package frc.robot.Commands;

import static frc.robot.Constants.DriveConstants.ROTATION_CONSTRAINTS;
import static frc.robot.Constants.DriveConstants.TRANSLATION_CONSTRAINTS;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;

/**
 * Command to drive to a pose.
 */
public class DriveToPose extends Command {
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;

  private final CommandSwerveDrivetrain drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;
  private final Supplier<Pose2d> goalPoseSupplier;

  private Pose2d goalPose;

  public static final StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("DriveState/targetPose", Pose2d.struct).publish();

  public DriveToPose(
      CommandSwerveDrivetrain drivetrainSubsystem,
      Supplier<Pose2d> poseProvider,
      Supplier<Pose2d> goalPoseSup) {
    this(drivetrainSubsystem, poseProvider, goalPoseSup, TRANSLATION_CONSTRAINTS, ROTATION_CONSTRAINTS);
  }

  public DriveToPose(
      CommandSwerveDrivetrain drivetrainSubsystem,
      Supplier<Pose2d> poseProvider,
      Supplier<Pose2d> goalPoseSup,
      TrapezoidProfile.Constraints xyConstraints,
      TrapezoidProfile.Constraints omegaConstraints) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.goalPoseSupplier = goalPoseSup;

    xController = new PIDController(DriveConstants.TRANSLATION_KP, DriveConstants.TRANSLATION_KI,
        DriveConstants.TRANSLATION_KD);
    yController = new PIDController(DriveConstants.TRANSLATION_KP, DriveConstants.TRANSLATION_KI,
        DriveConstants.TRANSLATION_KI);
    xController.setTolerance(DriveConstants.TRANLSATION_TOLLERANCE);
    yController.setTolerance(DriveConstants.TRANLSATION_TOLLERANCE);
    thetaController = new ProfiledPIDController(DriveConstants.ROTATION_KP, DriveConstants.ROTATION_KI,
        DriveConstants.ROTATION_KD, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(DriveConstants.ROTATION_TOLLERANCE));

    resetPIDControllers();
    
    if(goalPoseSupplier.get() == null){
      try {
        throw new Exception("How the hell is this null");
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
    goalPose = goalPoseSupplier.get();

    addRequirements(this.drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    resetPIDControllers();
    
    if(goalPoseSupplier.get() == null){
      try {
        throw new Exception("How the hell is this null");
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
    goalPose = goalPoseSupplier.get();

    thetaController.setGoal(goalPose.getRotation().getRadians());
    xController.setSetpoint(goalPose.getX());
    yController.setSetpoint(goalPose.getY());

    targetPosePublisher.set(goalPose);
  }

  public boolean atGoal() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atGoal() 
      && drivetrainSubsystem.getState().Speeds.vxMetersPerSecond < 0.1
      && drivetrainSubsystem.getState().Speeds.vyMetersPerSecond < 0.1
      && drivetrainSubsystem.getState().Speeds.omegaRadiansPerSecond < Units.degreesToRadians(5);
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    if(goalPose == null){
      initialize();
    }

    Pose2d robotPose = poseProvider.get();
    // Drive to the goal
    double xSpeed = xController.calculate(robotPose.getX());
    if (xController.atSetpoint()) {
      xSpeed = 0;
    }

    double ySpeed = yController.calculate(robotPose.getY());
    if (yController.atSetpoint()) {
      ySpeed = 0;
    }

    double omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }

    drivetrainSubsystem.setControl(DriveConstants.APPLY_FIELD_SPEEDS
        .withSpeeds(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed)));
      
    SmartDashboard.putNumber("DriveState/X", xSpeed);
    SmartDashboard.putNumber("DriveState/Y", ySpeed);
    SmartDashboard.putNumber("DriveState/Theta", omegaSpeed);
    SmartDashboard.putNumber("DriveState/GoalX", xController.getSetpoint());
    SmartDashboard.putNumber("DriveState/GoalY", yController.getSetpoint());
    SmartDashboard.putNumber("DriveState/GoalTheta", thetaController.getGoal().position);
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.setControl(DriveConstants.brake);
  }
}