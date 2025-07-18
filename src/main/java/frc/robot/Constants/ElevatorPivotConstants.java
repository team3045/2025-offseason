// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static frc.robot.Constants.ElevatorPivotConstants.maxHeight;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorPivotConstants {
    public static final int rightMotorId = 15;
    public static final int leftMotorId = 16;
    public static final int pivotMotorId = 17;
    public static final int pivotCancoderId = 16;
    public static final int canRangeId = 18;
    public static final int canRangeIdCoral = 19; // (?)

    public static final String canbus = "Canivore 3045";

    public static final String elevatorTable = "elevator";
    public static final String pivotTable = "pivot";

    public static final double numStages = 2;
    public static final double firstStageLength = Units.inchesToMeters(37.655020); // m
    public static final double secondStageLength = Units.inchesToMeters(11.993209); // m
    

    public static final double pivotArmLength = Units.inchesToMeters(14.961856); // m
    public static final double carriageHeightToPivot = Units.inchesToMeters(-1.00);
    public static final double minAngleDegrees = -180;
    public static final double maxAngleDegrees = 180;
    public static final double stowAngle = 0;
    public static final double intakingAngle = -90; // intaking coral
    public static final double processingAngle = -90;
    public static final double algaeReadyAngle = -120;
    public static final double algaeThrowAngle = -218;


    public static final double carriageToGround = Units.inchesToMeters(21.408345);// Units.inchesToMeters(16.752); //This is from the top of
                                                           // carriage to the ground, when at lowest position
    public static final double minimumHeight = carriageToGround; // m
    public static final double maxHeight = Units.inchesToMeters(81.427502); // m
    public static final double stowHeight = minimumHeight;
    public static final double intakingReadyHeight = minimumHeight + Units.inchesToMeters(7.1);
    public static final double intakingHeight = intakingReadyHeight;
    public static final double processingHeight = minimumHeight + Units.inchesToMeters(2);
    public static final double bargeHeight = Units.inchesToMeters(62.17);
    public static final double troughHeight = minimumHeight;
    public static final double climbHeight = stowHeight;
    public static final double algaeReadyHeight = minimumHeight + Units.inchesToMeters(30);
    public static final double algaeThrowHeight = bargeHeight;

    /* Collision */ // rewrite this
    // public static final double maxUpperCollisionAngle = 84;
    // public static final double maxalgaeCollisionAngle = 20;
    //public static final double coralTravelAngle = 
    //public static final double algaeTravelAngle = 90;
    //public static final double travelAngle = maxUpperCollisionAngle - 10;
    public static final double stageToCarriageMax = 0.15;
    //public static final double maxCollisionHeight = minimumHeight + 0.10;

    // new collision numberes that I actually understand
    public static final double coralOutcrop = Units.inchesToMeters(7); // has been rounded up
    public static final double algaeOutcrop = Units.inchesToMeters(9.5);
    public static final double highestBarrier = Units.inchesToMeters(0); // placeholder
    // anthing above clear heights are allowed
    public static final double nothingClearHeight = Units.inchesToMeters(7) + highestBarrier; 
    public static final double coralClearHeight = coralOutcrop + highestBarrier;
    public static final double algaeClearHeight = algaeOutcrop + highestBarrier;
    // 
    public static final double algaeTravelAngle = 90;
    public static final double coralTravelAngle = -100; 

    public static final double troughAngle = 6;
    public static final double bargeAngle = 70;


    public static final double rotorToSensorRatio = 1.0; // ?
    public static final double sensorToMechanismRatio = (3 / 1); 
    public static final double totalgaering = rotorToSensorRatio * sensorToMechanismRatio;

    public static final double pivotRotorToSensorRatio = (48.0 / 10.0) * (64.0 / 18.0) * (48.0 / 18.0); // ?
    public static final double pivotSensorToMechanismRatio = 1;
    public static final double pivotTotalgaering = pivotSensorToMechanismRatio * pivotRotorToSensorRatio;

    public static final double heightTolerance = 0.03; // 3cm
    public static final double angleToleranceDegrees = 5; // 3 degree tolerance
    public static final double safeMovementHeight = 0.830; // rewrite this out

    public static final double statorCurrentLimit = 80; // Amps
    public static final double supplyCurrentLimit = 80; // Amps
    public static final double supplyCurrentLimitLowerLimit = 60; // Amps
    public static final double supplyCurrentLimitLowerLimitTime = 1; // second
    public static final boolean statorCurrentLimitEnable = true;
    public static final boolean supplyCurrentLimitEnable = true;

    public static final double pivotStatorCurrentLimit = 60; // Amps
    public static final double pivotSupplyCurrentLimit = 30; // Amps
    public static final double pivotSupplyCurrentLimitLowerLimit = 30; // Amps
    public static final double pivotSupplyCurrentLimitLowerLimitTime = 1; // second
    public static final boolean pivotStatorCurrentLimitEnable = true;
    public static final boolean pivotSupplyCurrentLimitEnable = true;

    /* Simulation */
    public static final double carriageMass = 17.65; // <-- wrong

    public static final double drumRadius = Units.inchesToMeters(1); // m
    public static final double canvasWidth = 2; // m
    public static final double canvasHeight = 6; // m
    public static final double pivotMOI = 0.111571209; // moment of inertia Kg * m^2

    public static final double stage2StageLength = secondStageLength;
    public static final double verticalTimerThreshold = 0.5;

    public static final double pivotOffsetX = Units.inchesToMeters(-0.152);
    public static final double pivotOffsetY = Units.inchesToMeters(-0.165); 
    public static final double pivotOffsetZ = Units.inchesToMeters(-0.910);

    // Rotation of the output shaft. To get rotations of motor to the height of
    // elevator we need to multiply by the gear ratio
    public static final double rotationToLengthRatio = (2 * Math.PI * drumRadius) / 1; // 1.2566370614359172m / 1 rot
                                                                                       // //0.05729478

    public static final double maxAccelerationLinear = 4; // m per sec^2
    public static final double maxVelocityLinear = 4; // m per sec
    public static final double maxAccelerationRotations = maxAccelerationLinear / rotationToLengthRatio; // rot per
                                                                                                         // sec^2
    public static final double maxVelocityRotations = maxVelocityLinear / rotationToLengthRatio; // rot per sec

    public static final double pivotMaxAcceleration = 2; // rot per sec^2
    public static final double pivotMaxVelocity = 2; // rot per sec

    public static final double timesyncFrequency = 200; // Hz aka every 5 ms

    public static final InvertedValue leftInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue rightInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue pivotInvert = InvertedValue.CounterClockwise_Positive;

    public static final double kP = 16.3;
    public static final double kI = 0;
    public static final double kD = 0.5;
    public static final double kG = 0.35;
    public static final double kS = 0.12;
    public static final double kA = 0;
    public static final double kV = 0.76;

    public static final double pivotKP = !Utils.isSimulation() ? 30 : 40; // these must be tuned
    public static final double pivotKI = !Utils.isSimulation() ? 0 : 0;
    public static final double pivotKD = !Utils.isSimulation() ? 0 : 0;
    public static final double pivotKG = !Utils.isSimulation() ? 0.4 : 0.65;
    public static final double pivotKS = !Utils.isSimulation() ? 0.08 : 0;
    public static final double pivotKA = !Utils.isSimulation() ? 0 : 0.02;
    public static final double pivotKV = !Utils.isSimulation() ? 2.8 : 0.55;

    public static final double magnetOffset = !Utils.isSimulation() ? 0.190430 : 0; // I don't even know what this is
    public static final SensorDirectionValue pivotEncoderSensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    public enum HeightPositions {
        L4(Units.inchesToMeters(60.25)),
        L3(Units.inchesToMeters(22.3)),
        L2(Units.inchesToMeters(6.3)),
        L4_AUTO(Units.inchesToMeters(60.25)), // this may or may not be redundant
        L4_V2(1.92), // this may or may not be redundant
        LOW_algae(Units.inchesToMeters(21.91)),
        HIGH_algae(Units.inchesToMeters(37.91));

        private final double height;

        HeightPositions(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }

    public enum AnglePositions {
        L4(140),
        L3(100.75),
        L2(100.75),
        L4_AUTO(140), // again these may be redundant
        L4_V2(140),
        LOW_algae(-90),
        HIGH_algae(-90);

        private final double angle;

        AnglePositions(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }

    /* Configuration */
    public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(statorCurrentLimit)
            .withSupplyCurrentLimit(supplyCurrentLimit)
            .withStatorCurrentLimitEnable(statorCurrentLimitEnable)
            .withSupplyCurrentLimitEnable(supplyCurrentLimitEnable)
            .withSupplyCurrentLowerTime(supplyCurrentLimitLowerLimitTime)
            .withSupplyCurrentLowerLimit(supplyCurrentLimitLowerLimit);

    public static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .withRotorToSensorRatio(rotorToSensorRatio)
            .withSensorToMechanismRatio(sensorToMechanismRatio);

    public static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(maxAccelerationRotations)
            .withMotionMagicCruiseVelocity(maxVelocityRotations); // Consider adding jerk or making it expo

    public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
            .withControlTimesyncFreqHz(timesyncFrequency)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final Slot0Configs slot0Configs = new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKA(kA)
            .withKD(kD)
            .withKG(kG)
            .withKI(kI)
            .withKP(kP)
            .withKS(kS)
            .withKV(kV)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withFeedback(feedbackConfigs)
            .withMotionMagic(motionMagicConfigs)
            .withMotorOutput(motorOutputConfigs)
            .withSlot0(slot0Configs);

    public static final CurrentLimitsConfigs pivotCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(pivotStatorCurrentLimit)
            .withSupplyCurrentLimit(pivotSupplyCurrentLimit)
            .withStatorCurrentLimitEnable(pivotStatorCurrentLimitEnable)
            .withSupplyCurrentLimitEnable(pivotSupplyCurrentLimitEnable)
            .withSupplyCurrentLowerTime(pivotSupplyCurrentLimitLowerLimitTime)
            .withSupplyCurrentLowerLimit(pivotSupplyCurrentLimitLowerLimit);

    public static final FeedbackConfigs pivotFeedbackConfigs = new FeedbackConfigs()
            .withFusedCANcoder(new CoreCANcoder(pivotCancoderId, canbus))
            .withRotorToSensorRatio(pivotRotorToSensorRatio)
            .withSensorToMechanismRatio(pivotSensorToMechanismRatio);

    public static final MotionMagicConfigs pivotMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(pivotMaxAcceleration)
            .withMotionMagicCruiseVelocity(pivotMaxVelocity); // Consider adding jerk or making it expo

    public static final MotorOutputConfigs pivotMotorOutputConfigs = new MotorOutputConfigs()
            .withControlTimesyncFreqHz(timesyncFrequency)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(pivotInvert);

    public static final Slot0Configs pivotSlot0Configs = new Slot0Configs()
            .withKA(pivotKA)
            .withKD(pivotKD)
            .withKG(pivotKG)
            .withKI(pivotKI)
            .withKP(pivotKP)
            .withKS(pivotKS)
            .withKV(pivotKV)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    public static final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(pivotCurrentLimits)
            .withFeedback(pivotFeedbackConfigs)
            .withMotionMagic(pivotMotionMagicConfigs)
            .withMotorOutput(pivotMotorOutputConfigs)
            .withSlot0(pivotSlot0Configs);

    public static final CANcoderConfiguration pivotCancoderConfig = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                    .withSensorDirection(pivotEncoderSensorDirection)
                    .withAbsoluteSensorDiscontinuityPoint(0.5) // [-0.5,0.5]
                    .withMagnetOffset(magnetOffset));

    public static final double fovCenterX = 0;
    public static final double fovCenterY = 0;
    public static final double fovRangeX = 6.75;
    public static final double fovRangeY = 6.75;

    public static final double minSignalStrength = 0;
    public static final double proximityHysterisis = 0.01; // 1 cm
    public static final double proximityThreshold = 0.08; // 8cm

    public static final double updateFrequency = 50; // every 20 ms, this is overridden to 100 Hz when we're in
                                                     // ShortRange100hz mode
    public static final double debounceTime = 0.4;

    public static final FovParamsConfigs fovConfigs = new FovParamsConfigs()
            .withFOVCenterX(fovCenterX)
            .withFOVCenterY(fovCenterY)
            .withFOVRangeX(fovRangeX)
            .withFOVRangeY(fovRangeY);

    public static final ProximityParamsConfigs proximityConfigs = new ProximityParamsConfigs()
            .withMinSignalStrengthForValidMeasurement(minSignalStrength)
            .withProximityHysteresis(proximityHysterisis)
            .withProximityThreshold(proximityThreshold);

    public static final ToFParamsConfigs tofConfigs = new ToFParamsConfigs()
            .withUpdateFrequency(updateFrequency)
            .withUpdateMode(UpdateModeValue.ShortRange100Hz);

    public static final CANrangeConfiguration canRangeConfig = new CANrangeConfiguration()
            .withFovParams(fovConfigs)
            .withProximityParams(proximityConfigs)
            .withToFParams(tofConfigs);



    // must be tuned
   public static final double coralFovCenterX = 0; 
   public static final double coralFovCenterY = 0;
   public static final double coralFovRangeX = 6.75;
   public static final double coralFovRangeY = 6.75;
        
   public static final double coralMinSignalStrength = 0;
   public static final double coralProximityHysterisis = 0.01; // 1 cm
   public static final double coralProximityThreshold = 0.08; // 8cm


    public static final FovParamsConfigs coralFovConfigs = new FovParamsConfigs()
            .withFOVCenterX(coralFovCenterX)
            .withFOVCenterY(coralFovCenterY)
            .withFOVRangeX(coralFovRangeX)
            .withFOVRangeY(coralFovRangeY);

    public static final ProximityParamsConfigs coralProximityConfigs = new ProximityParamsConfigs()
            .withMinSignalStrengthForValidMeasurement(coralMinSignalStrength)
            .withProximityHysteresis(coralProximityHysterisis)
            .withProximityThreshold(coralProximityThreshold);

    public static final CANrangeConfiguration coralRangeConfig = new CANrangeConfiguration()
            .withFovParams(coralFovConfigs)
            .withProximityParams(coralProximityConfigs)
            .withToFParams(tofConfigs);
}