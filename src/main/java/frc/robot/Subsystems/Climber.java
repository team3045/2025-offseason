package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commons.GremlinLogger;
import frc.robot.commons.GremlinUtil;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import static frc.robot.Constants.ClimberConstants.*;

import java.io.ObjectInputFilter.Config;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class Climber extends SubsystemBase {
    private TalonFX pivotMotor = new TalonFX(pivotID, CAN_STRING);
    private TalonFX wingsMotor = new TalonFX(wingsID, CAN_STRING);

    private CANcoder pivotCancoder = new CANcoder(pivotCancoderID, CAN_STRING);

    private double targetAngleDegrees;
    private double voltage; 

    public Trigger atTargetAngle = new Trigger(() -> atTargetAngle()); // define
    
    public Climber() {
        configDevices(); // define

        if (Utils.isSimulation()) {
            configSimulation(); // define
        }

        targetAngleDegrees = getAngleDegrees(); // define
    }

    public void configDevices() {
        pivotMotor.getConfigurator().apply(pivotMotorConfig); // define
        wingsMotor.getConfigurator().apply(wingMotorConfig); // define
        pivotCancoder.getConfigurator().apply(pivotCancoderConfig); // define

        pivotMotor.clearStickyFaults();
        wingsMotor.clearStickyFaults();

        BaseStatusSignal.setUpdateFrequencyForAll(200,
            pivotMotor.getPosition(),
            wingsMotor.getPosition(),
            pivotCancoder.getPosition());
    }

    public double getangleRotations() {
        return pivotCancoder.getPosition().getValueAsDouble();
    }

    public double getAngleDegrees() {
        return Units.rotationsToDegrees(pivotCancoder.getPosition().getValueAsDouble());
    }

    public double getAngleRadians() {
        return Units.rotationsToRadians(pivotCancoder.getPosition().getValueAsDouble());
    }

    public boolean atTargetAngle() {
        return Math.abs(getAngleDegrees() - targetAngleDegrees) < angleToleranceDegrees; 
    }

    private void setAngleTargetDegrees(double targetAngleDegrees) {
        this.targetAngleDegrees = GremlinUtil.clampWithLogs(maxAngle, minAngle, targetAngleDegrees);

        double targetAngleRotations = Units.degreesToRadians(targetAngleDegrees);

        MotionMagicVoltage request = new MotionMagicVoltage(targetAngleRotations)
            .withEnableFOC(true)
            .withSlot(0)
            .withUpdateFreqHz(1000);
        
        pivotMotor.setControl(request);
    }



    // commands
    public Command goToAngleDegrees(DoubleSupplier angle) {
        return this.run(() -> {
            setAngleTargetDegrees(angle.getAsDouble());
        }).until(atTargetAngle);
    }

    public Command goToAngleDegrees(double angle) {
        return this.run(() -> {
            setAngleTargetDegrees(angle);
        }).until(atTargetAngle);
    }

    public Command max() {
        return goToAngleDegrees(maxAngle);
    }
    public Command min() {
        return goToAngleDegrees(minAngle);
    }

    public Command increaseVoltage() {
        return this.runOnce(() -> {
            voltage += 0.1; 
            pivotMotor.setVoltage(voltage);
            SmartDashboard.putNumber("voltage", voltage);
        });
    }
    public Command decreaseVoltage() {
        return this.runOnce(() -> {
            voltage -= 0.1; 
            pivotMotor.setVoltage(voltage);
            SmartDashboard.putNumber("voltage", voltage);
        });
    }
    public Command zeroVoltage() {
        return this.runOnce(() -> {
            voltage = 0; 
            pivotMotor.setVoltage(voltage);
            SmartDashboard.putNumber("voltage", voltage);
        }); 
    }
    public Command applyDownwardCurrent() {
        return this.runOnce(() ->  {
            double current = -10;
            pivotMotor.setControl(new TorqueCurrentFOC(current));
        });
    }
    public Command applyUpwardCurrent() {
        return this.runOnce(() -> {
            double current = 10;
            pivotMotor.setControl(new TorqueCurrentFOC(current));
        });
    }
    public Command zeroCurrent() {
        return this.runOnce(() -> {
            double current = 0; 
            pivotMotor.setControl(new TorqueCurrentFOC(current));
        });
    }

    public Command intakeCage() {
        return this.runOnce(() -> {
            double current = 10;
            wingsMotor.setControl(new TorqueCurrentFOC(current));
        });
    }
    public Command ejectCage() {
        return this.runOnce(() -> {
            double current = -10;
            wingsMotor.setControl(new TorqueCurrentFOC(current));
        });
    }
    public Command stopWingsMotor() {
        return this.runOnce(() -> {
            wingsMotor.setControl(new TorqueCurrentFOC(0));
        });
    }

    public Command beginClimbSequence() {
        return this.runOnce(() -> 
            max()
            .andThen(intakeCage())
        );
    }

    @Override
    public void periodic() {
        updateMechanism2d(); // define 
        
        SmartDashboard.putNumber("pivot angle", getAngleDegrees());
    }


    // sim
    private final FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 
        flywheelMOI, // define
        flywheelGearing), // define
    DCMotor.getKrakenX60(1)); 

    private final SingleJointedArmSim singleJointedArmSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1),
        pivotGearing, // define
        pivotMOI, // define
        climberLength, // define
        Units.degreesToRadians(minAngle),
        Units.degreesToRadians(maxAngle),
        true,
        Units.degreesToRadians(minAngle)
    ); 
    private TalonFXSimState wingsMotorSim;
    private TalonFXSimState pivotMotorSim;
    private CANcoderSimState pivotCancoderSim; 

    private Mechanism2d pivotMech = new Mechanism2d(canvasWidth, canvasHeight); // define both of these
    private StructArrayPublisher<Pose3d> componentPosesPublisher = NetworkTableInstance.getDefault()
        .getTable(climberTable)
        .getStructArrayTopic("componentPoses", Pose3d.struct).publish();
    
    private MechanismRoot2d pivotRoot = pivotMech.getRoot(("pivotRoot"), 2, 3); // no clue what the numbers are
    private MechanismLigament2d pivotLigament = pivotRoot.append(new MechanismLigament2d("climberLigament", climberLength, minAngle));
    
    public void configSimulation() {
        wingsMotorSim = wingsMotor.getSimState();
        pivotMotorSim = pivotMotor.getSimState();
        pivotCancoderSim = pivotCancoder.getSimState();

        wingsMotorSim.Orientation = ChassisReference.CounterClockwise_Positive; // maybe fix
        pivotMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;

        singleJointedArmSim.setState(0,0);
    }

    @Override
    public void simulationPeriodic() {
        wingsMotorSim = wingsMotor.getSimState();
        pivotMotorSim = pivotMotor.getSimState();
        pivotCancoderSim = pivotCancoder.getSimState();

        wingsMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        pivotCancoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        pivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        flywheelSim.setInputVoltage(pivotMotorSim.getMotorVoltageMeasure().in(Volts));
        flywheelSim.update(0.02);

        singleJointedArmSim.setInputVoltage(pivotMotorSim.getMotorVoltageMeasure().in(Volts));
        singleJointedArmSim.update(0.02);


        pivotCancoderSim.setRawPosition(edu.wpi.first.math.util.Units.radiansToRotations((singleJointedArmSim.getAngleRads() * pivotSensorToMechanismRatio))); 
        pivotCancoderSim.setVelocity(edu.wpi.first.math.util.Units.radiansToRotations(singleJointedArmSim.getVelocityRadPerSec() * pivotSensorToMechanismRatio)); 
        pivotMotorSim.setRawRotorPosition(edu.wpi.first.math.util.Units.radiansToRotations(singleJointedArmSim.getAngleRads() * pivotTotalGearing));
        pivotMotorSim.setRotorVelocity(edu.wpi.first.math.util.Units.radiansToRotations(singleJointedArmSim.getVelocityRadPerSec() * pivotTotalGearing));
            
        updateMechanism2d();
    }

    public void updateMechanism2d() {
        double currentAngle = getAngleDegrees();

        if (GremlinLogger.DEBUG) { // ON BY DEFAULT
            pivotLigament.setAngle(180 - currentAngle); // I have a vague idea of what this is for

            componentPosesPublisher.set(new Pose3d[] {
                new Pose3d(pivotOffsetX, 
                            pivotOffsetY, 
                            pivotOffsetZ, 
                            new Rotation3d(0, 
                                            -getAngleRadians(), // same reason as the 180 - probably
                                            0))
            });
        }

        GremlinLogger.log("Climber/Pivot", new Pose3d[] {
            new Pose3d(pivotOffsetX, 
                        pivotOffsetY,
                        pivotOffsetZ, 
                        new Rotation3d(0, 
                                        -getAngleRadians(), // same reason as the 180 - probably
                                        0))
        });
    }

    private final SysIdRoutine m_SysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.5).per(Second),
            Volts.of(2),
            null,
            (state) -> SignalLogger.writeString("state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> {
                pivotMotor.setVoltage((volts.in(Volts)));
            }, 
            null, 
            this)
        );
                
        
        // possibly redundant
    public Command sysIdQuasistatic(SysIdRoutine.Direction d) {
        return m_SysIdRoutine.quasistatic(d);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction d) {
        return m_SysIdRoutine.dynamic(d);
    }
}
