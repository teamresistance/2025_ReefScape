// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commandgroups.ElevatorCommandGroup;
import frc.robot.commands.ChooseReefCmd;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FlipperGripperCmd;
import frc.robot.commands.FlipperScoreCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlipperSubsystem;
import frc.robot.subsystems.PhysicalReefInterfaceSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LoggedTunableNumber;
import java.io.IOException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final LoggedTunableNumber pathFindingThreshold =
      new LoggedTunableNumber("Drive/SWITCH_DISTANCE_THRESHOLD", 0.7);
  public final PhotonCamera frontLeftCamera = new PhotonCamera("front-left");
  public final PhotonCamera frontRightCamera = new PhotonCamera("front-right");
  public final PhotonCamera backLeftCamera = new PhotonCamera("back_left");
  public final PhotonCamera backRightCamera = new PhotonCamera("back_right");
  public final PhotonCamera frontCenterCamera = new PhotonCamera("front_center");
  // Subsystems
  private final Drive drive;
  private final FlipperSubsystem m_flipperSubsystem = new FlipperSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  private final Joystick joystick2 = new Joystick(1);
  private final Joystick coJoystick = new Joystick(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  public Vision aprilTagVision;
  // Create the target Transform2d (Translation and Rotation)
  Translation2d targetTranslation = new Translation2d(14.35, 4.31); // X = 14, Y = 4
  Rotation2d targetRotation = new Rotation2d(Units.degreesToRadians(-178.0)); // No rotation
  Transform2d targetTransform = new Transform2d(targetTranslation, targetRotation);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive = configureDrive();
    autoChooser = configureAutos();
    aprilTagVision = configureAprilTagVision();
    configureButtonBindings();
  }

  private LoggedDashboardChooser<Command> configureAutos() {
    // Set up auto routines
    LoggedDashboardChooser<Command> autoChooser =
        new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    return autoChooser;
  }

  private Vision configureAprilTagVision() {
    try {
      aprilTagVision =
          new Vision(
              frontLeftCamera,
              frontRightCamera,
              backLeftCamera,
              backRightCamera,
              frontCenterCamera);
    } catch (IOException e) {
      e.printStackTrace();
    }
    aprilTagVision.setDataInterfaces(drive::getPose, drive::addAutoVisionMeasurement);
    return aprilTagVision;
  }

  private Drive configureDrive() {
    // Real robot, instantiate hardware IO implementations
    // Sim robot, instantiate physics sim IO implementations
    // Replayed robot, disable IO implementations
    return switch (Constants.CurrentMode) {
      case REAL ->
          // Real robot, instantiate hardware IO implementations
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOTalonFX(TunerConstants.FrontLeft),
              new ModuleIOTalonFX(TunerConstants.FrontRight),
              new ModuleIOTalonFX(TunerConstants.BackLeft),
              new ModuleIOTalonFX(TunerConstants.BackRight));
      case SIM ->
          // Sim robot, instantiate physics sim IO implementations
          new Drive(
              new GyroIO() {},
              new ModuleIOSim(TunerConstants.FrontLeft),
              new ModuleIOSim(TunerConstants.FrontRight),
              new ModuleIOSim(TunerConstants.BackLeft),
              new ModuleIOSim(TunerConstants.BackRight));
      default ->
          // Replayed robot, disable IO implementations
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    };
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link
   * XboxController}), and then passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Lock to 0° when A button is held
    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    driver
        .leftBumper()
        .whileTrue(
            DriveCommands.goToTransformWithPathFinder(drive, targetTransform)
                .beforeStarting(
                    () ->
                        DriveCommands.goToTransformWithPathFinder(drive, targetTransform)
                            .cancel()));

    driver
        .rightBumper()
        .whileTrue(
            DriveCommands.goToTransformWithPathFinder(drive, targetTransform)
                .beforeStarting(
                    () -> {
                      DriveCommands.goToTransform(drive, targetTransform).cancel();
                      DriveCommands.goToTransformWithPathFinder(drive, targetTransform).cancel();
                    }));

    // Reset gyro to 0 when B button is pressed
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    //    Codriver Bindings

    final PhysicalReefInterfaceSubsystem m_PhysicalReefSubsystem =
        new PhysicalReefInterfaceSubsystem();
    // execute
    new JoystickButton(coJoystick, 1)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, -1, -1, true));
    // level
    new JoystickButton(coJoystick, 2)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, 0, -1, -1, false));
    new JoystickButton(coJoystick, 3)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, 1, -1, -1, false));
    new JoystickButton(coJoystick, 4)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, 2, -1, -1, false));
    new JoystickButton(coJoystick, 6)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, 3, -1, -1, false));
    // pos
    new JoystickButton(coJoystick, 7)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, 0, -1, false));
    new JoystickButton(coJoystick, 8)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, 1, -1, false));
    new JoystickButton(coJoystick, 9)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, 2, -1, false));
    new JoystickButton(coJoystick, 10)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, 3, -1, false));
    new JoystickButton(coJoystick, 11)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, 4, -1, false));
    new JoystickButton(coJoystick, 12)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, 5, -1, false));
    // rightleft
    new JoystickButton(coJoystick, 5)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, -1, 1, false));

    //
    //    Standard Joystick Bindings
    // not sure if these should be joystick2
    new JoystickButton(joystick2, 1).onTrue(new FlipperScoreCmd(m_flipperSubsystem));
    new JoystickButton(joystick2, 5).onTrue(new FlipperGripperCmd(m_flipperSubsystem));
    new JoystickButton(joystick2, 3).onTrue(new ElevatorCommandGroup(m_elevatorSubsystem, 0));
    new JoystickButton(joystick2, 4).onTrue(new ElevatorCommandGroup(m_elevatorSubsystem, 1));
    new JoystickButton(joystick2, 6).onTrue(new ElevatorCommandGroup(m_elevatorSubsystem, 2));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
