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
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
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
public class ModuleIOTalonD_SparkMaxT implements ModuleIO {
  // Gear ratios for WCP SwerveX, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = Constants.DRIVE_GEAR_RATIO;
  private static final double TURN_GEAR_RATIO = Constants.TURN_GEAR_RATIO;

  private final TalonFX driveTalon;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder turnRelativeEncoder;
  private final StatusSignal<Double> turnAbsolutePosition;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;
  private final StatusSignal<Double> driveTemperature;

  private final Queue<Double> timestampQueueSparkMax;
  private final Queue<Double> timestampQueueTalon;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  private final CANcoder cancoder;

  public ModuleIOTalonD_SparkMaxT(int index) {
    switch (index) {
      case 0: // FL
        driveTalon = new TalonFX(Constants.DRIVE_SPARK_MAX_FL);
        turnSparkMax = new CANSparkMax(Constants.TURN_SPARK_MAX_FL, MotorType.kBrushless);
        cancoder = new CANcoder(Constants.CANCODER_FL);
        absoluteEncoderOffset =
            new Rotation2d(Units.rotationsToRadians(Constants.ABSOLUTE_ENCODER_OFFSET_FL));
        break;
      case 1: // FR
        driveTalon = new TalonFX(Constants.DRIVE_SPARK_MAX_FR);
        turnSparkMax = new CANSparkMax(Constants.TURN_SPARK_MAX_FR, MotorType.kBrushless);
        cancoder = new CANcoder(Constants.CANCODER_FR);
        absoluteEncoderOffset =
            new Rotation2d(Units.rotationsToRadians(Constants.ABSOLUTE_ENCODER_OFFSET_FR));
        break;
      case 2: // BL
        driveTalon = new TalonFX(Constants.DRIVE_SPARK_MAX_BL);
        turnSparkMax = new CANSparkMax(Constants.TURN_SPARK_MAX_BL, MotorType.kBrushless);
        cancoder = new CANcoder(Constants.CANCODER_BL);
        absoluteEncoderOffset =
            new Rotation2d(Units.rotationsToRadians(Constants.ABSOLUTE_ENCODER_OFFSET_BL));
        break;
      case 3: // BR
        driveTalon = new TalonFX(Constants.DRIVE_SPARK_MAX_BR);
        turnSparkMax = new CANSparkMax(Constants.TURN_SPARK_MAX_BR, MotorType.kBrushless);
        cancoder = new CANcoder(Constants.CANCODER_BR);
        absoluteEncoderOffset =
            new Rotation2d(Units.rotationsToRadians(Constants.ABSOLUTE_ENCODER_OFFSET_BR));
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnSparkMax.restoreFactoryDefaults();
    turnSparkMax.setCANTimeout(250);
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnSparkMax.setInverted(isTurnMotorInverted);
    turnSparkMax.setSmartCurrentLimit(50);
    turnSparkMax.enableVoltageCompensation(12.0);
    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);
    turnSparkMax.setCANTimeout(0);

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
    driveConfig.CurrentLimits.StatorCurrentLimit = 100.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    turnSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    timestampQueueSparkMax = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    timestampQueueTalon = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value =
                      cancoder.getAbsolutePosition().getValue()
                          - absoluteEncoderOffset.getRotations();
                  if (turnSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();
    driveTemperature = driveTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(Module.ODOMETRY_FREQUENCY, drivePosition);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition);
    turnSparkMax.burnFlash();
    driveTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    BaseStatusSignal.refreshAll(turnAbsolutePosition, drivePosition);
    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveVelocity.getValueAsDouble())
            / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

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
        timestampQueueSparkMax.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);

    inputs.temperatureDrive = driveTemperature.getValue();
    inputs.temperatureTurn = turnSparkMax.getMotorTemperature();
    timestampQueueSparkMax.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
