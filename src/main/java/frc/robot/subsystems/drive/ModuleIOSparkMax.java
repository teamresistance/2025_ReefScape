/*
 * MIT License
 *
 * Copyright (c) 2025 Team 86
 *
 * https://github.com/teamresistance
 *
 * More details provided in license files
 */

 package frc.robot.subsystems.drive;

 import com.ctre.phoenix6.BaseStatusSignal;
 import com.ctre.phoenix6.StatusSignal;
 import com.ctre.phoenix6.configs.CANcoderConfiguration;
 import com.ctre.phoenix6.hardware.CANcoder;
 import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
 import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
 import com.revrobotics.spark.SparkLowLevel.MotorType;
 import com.revrobotics.REVLibError;
 import com.revrobotics.spark.SparkMax;
 import com.revrobotics.RelativeEncoder;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.util.Units;
 import frc.robot.Constants;
 
 import static edu.wpi.first.units.Units.Degrees;
 
 import java.util.OptionalDouble;
 import java.util.Queue;
 
 /**
  * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
  * or NEO 550), and analog absolute encoder connected to the RIO
  *
  * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
  * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
  *
  * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
  * motion on the drive motor will propel the robot forward) and copy the reported values from the
  * absolute encoders using AdvantageScope. These values are logged under
  * "/Drive/ModuleX/TurnAbsolutePositionRad"
  */
 public class ModuleIOSparkMax implements ModuleIO {
   // Gear ratios for WCP SwerveX, adjust as necessary
   private static final double DRIVE_GEAR_RATIO = Constants.DRIVE_GEAR_RATIO;
   private static final double TURN_GEAR_RATIO = Constants.TURN_GEAR_RATIO;
 
   private final SparkMax driveSparkMax;
   private final SparkMax turnSparkMax;
 
   private final RelativeEncoder driveEncoder;
   private final RelativeEncoder turnRelativeEncoder;
   private final StatusSignal<Double> turnAbsolutePosition;
   private final Queue<Double> timestampQueue;
   private final Queue<Double> drivePositionQueue;
   private final Queue<Double> turnPositionQueue;
 
   private final boolean isTurnMotorInverted = true;
   private final Rotation2d absoluteEncoderOffset;
 
   private final CANcoder cancoder;
 
   public ModuleIOSparkMax(int index) {
     switch (index) {
       case 0: // FL
         driveSparkMax = new SparkMax(Constants.DRIVE_SPARK_MAX_FL, MotorType.kBrushless);
         turnSparkMax = new SparkMax(Constants.TURN_SPARK_MAX_FL, MotorType.kBrushless);
         cancoder = new CANcoder(Constants.CANCODER_FL);
         absoluteEncoderOffset =
             new Rotation2d(Units.rotationsToRadians(Constants.ABSOLUTE_ENCODER_OFFSET_FL));
         break;
       case 1: // FR
         driveSparkMax = new SparkMax(Constants.DRIVE_SPARK_MAX_FR, MotorType.kBrushless);
         driveSparkMax.setInverted(true); // this does not matter, gets reset anyways
         turnSparkMax = new SparkMax(Constants.TURN_SPARK_MAX_FR, MotorType.kBrushless);
         cancoder = new CANcoder(Constants.CANCODER_FR);
         absoluteEncoderOffset =
             new Rotation2d(Units.rotationsToRadians(Constants.ABSOLUTE_ENCODER_OFFSET_FR));
         break;
       case 2: // BL
         driveSparkMax = new SparkMax(Constants.DRIVE_SPARK_MAX_BL, MotorType.kBrushless);
         turnSparkMax = new SparkMax(Constants.TURN_SPARK_MAX_BL, MotorType.kBrushless);
         cancoder = new CANcoder(Constants.CANCODER_BL);
         absoluteEncoderOffset =
             new Rotation2d(Units.rotationsToRadians(Constants.ABSOLUTE_ENCODER_OFFSET_BL));
         break;
       case 3: // BR
         driveSparkMax = new SparkMax(Constants.DRIVE_SPARK_MAX_BR, MotorType.kBrushless);
         turnSparkMax = new SparkMax(Constants.TURN_SPARK_MAX_BR, MotorType.kBrushless);
         cancoder = new CANcoder(Constants.CANCODER_BR);
         absoluteEncoderOffset =
             new Rotation2d(Units.rotationsToRadians(Constants.ABSOLUTE_ENCODER_OFFSET_BR));
         break;
       default:
         throw new RuntimeException("Invalid module index");
     }
 
     cancoder.getConfigurator().apply(new CANcoderConfiguration());
 
     turnAbsolutePosition = cancoder.getAbsolutePosition().getValueAsDouble();
 
     // driveSparkMax.restoreFactoryDefaults(); // got reset here
     // turnSparkMax.restoreFactoryDefaults();
 
     driveSparkMax.setCANTimeout(250);
     turnSparkMax.setCANTimeout(250);
 
     driveEncoder = driveSparkMax.getEncoder();
     turnRelativeEncoder = turnSparkMax.getEncoder();
 
     turnSparkMax.setInverted(isTurnMotorInverted);
     // driveSparkMax.setSmartCurrentLimit(50);
     // turnSparkMax.setSmartCurrentLimit(50);
     // driveSparkMax.enableVoltageCompensation(12.0);
     // turnSparkMax.enableVoltageCompensation(12.0);
 
     driveEncoder.setPosition(0.0);
     // driveEncoder.setMeasurementPeriod(10);
     // driveEncoder.setAverageDepth(2);
 
     turnRelativeEncoder.setPosition(0.0);
     // turnRelativeEncoder.setMeasurementPeriod(10);
     // turnRelativeEncoder.setAverageDepth(2);
 
     driveSparkMax.setCANTimeout(0);
     turnSparkMax.setCANTimeout(0);
 
     driveSparkMax.setPeriodicFrameTimeout((int) (1000.0 / Module.ODOMETRY_FREQUENCY)); //Changed from setPeriodicFrameTimeout with PeriodicFrame.kStatus2
     turnSparkMax.setPeriodicFrameTimeout((int) (1000.0 / Module.ODOMETRY_FREQUENCY));
     timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
     drivePositionQueue =
         SparkMaxOdometryThread.getInstance()
             .registerSignal(
                 () -> {
                   double value = driveEncoder.getPosition();
                   if (driveSparkMax.getLastError() == REVLibError.kOk) {
                     return OptionalDouble.of(value);
                   } else {
                     return OptionalDouble.empty();
                   }
                 });
     turnPositionQueue =
         SparkMaxOdometryThread.getInstance()
             .registerSignal(
                 () -> {
                   double value =
                       cancoder.getAbsolutePosition().getValueAsDouble()
                           - absoluteEncoderOffset.getRotations();
                   if (driveSparkMax.getLastError() == REVLibError.kOk) {
                     return OptionalDouble.of(value);
                   } else {
                     return OptionalDouble.empty();
                   }
                 });
 
     BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition);
     // driveSparkMax.burnFlash();
     // turnSparkMax.burnFlash();
     //TODO: Figure out  how to burn flash
   }
 
   @Override
   public void updateInputs(ModuleIOInputs inputs) {
 
     BaseStatusSignal.refreshAll(turnAbsolutePosition);
     inputs.drivePositionRad =
         Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
     inputs.driveVelocityRadPerSec =
         Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
     inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
     inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};
 
     inputs.turnAbsolutePosition =
         Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
             .minus(absoluteEncoderOffset);
     inputs.turnPosition =
         Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
     inputs.turnVelocityRadPerSec =
         Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
             / TURN_GEAR_RATIO;
     inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
     inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
 
     inputs.odometryTimestamps =
         timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
     inputs.odometryDrivePositionsRad =
         drivePositionQueue.stream()
             .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
             .toArray();
     inputs.odometryTurnPositions =
         turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
 
     inputs.temperatureDrive = driveSparkMax.getMotorTemperature();
     inputs.temperatureTurn = turnSparkMax.getMotorTemperature();
     timestampQueue.clear();
     drivePositionQueue.clear();
     turnPositionQueue.clear();
   }
 
   @Override
   public void setDriveVoltage(double volts) {
     driveSparkMax.setVoltage(volts);
   }
 
   @Override
   public void setTurnVoltage(double volts) {
     turnSparkMax.setVoltage(volts);
   }
 
   @Override
   public void setDriveBrakeMode(boolean enable) {
     driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
   }
 
   @Override
   public void setTurnBrakeMode(boolean enable) {
     turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
   }
 }
 