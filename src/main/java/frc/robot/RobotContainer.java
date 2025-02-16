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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final FlipperSubsystem m_flipperSubsystem = new FlipperSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  final double DEADZONE = 0.1;

  // Create the target Transform2d (Translation and Rotation)
  Translation2d targetTranslation = new Translation2d(15, 4); // X = 14, Y = 4
  Rotation2d targetRotation = new Rotation2d(0.0); // No rotation
  Transform2d targetTransform = new Transform2d(targetTranslation, targetRotation);

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  // private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick Joystick1 = new Joystick(0);
  private final Joystick Joystick2 = new Joystick(1);
  private final Joystick CoJoystick = new Joystick(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -applyDeadband(controller.getLeftY()),
            () -> -applyDeadband(controller.getLeftX()),
            () -> -applyDeadband(controller.getRightX())));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -applyDeadband(controller.getLeftY()),
                () -> -applyDeadband(controller.getLeftX()),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller.leftBumper().whileTrue(DriveCommands.goToTransform(drive, targetTransform));

    // **Left Trigger - Go to AprilTag Position A**
    controller
        .leftTrigger()
        .whileTrue(DriveCommands.goTo2DPos(drive, 0.0, 1.0, 0.0)); // Example values

    // **Right Trigger - Go to AprilTag Position B**
    controller
        .rightTrigger()
        .whileTrue(DriveCommands.goTo2DPos(drive, 1.0, 2.0, 0.0)); // Example values
    //
    //    Codriver Bindings
    //
    final PhysicalReefInterfaceSubsystem m_PhysicalReefSubsystem =
        new PhysicalReefInterfaceSubsystem();
    // execute
    new JoystickButton(CoJoystick, 1)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, -1, -1, true));
    // level
    new JoystickButton(CoJoystick, 2)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, 0, -1, -1, false));
    new JoystickButton(CoJoystick, 3)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, 1, -1, -1, false));
    new JoystickButton(CoJoystick, 4)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, 2, -1, -1, false));
    new JoystickButton(CoJoystick, 6)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, 3, -1, -1, false));
    // pos
    new JoystickButton(CoJoystick, 7)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, 0, -1, false));
    new JoystickButton(CoJoystick, 8)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, 1, -1, false));
    new JoystickButton(CoJoystick, 9)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, 2, -1, false));
    new JoystickButton(CoJoystick, 10)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, 3, -1, false));
    new JoystickButton(CoJoystick, 11)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, 4, -1, false));
    new JoystickButton(CoJoystick, 12)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, 5, -1, false));
    // rightleft
    new JoystickButton(CoJoystick, 5)
        .onTrue(new ChooseReefCmd(m_PhysicalReefSubsystem, -1, -1, 1, false));

    //
    //    Standard Joystick Bindings
    //
    new JoystickButton(Joystick1, 1).onTrue(new FlipperScoreCmd(m_flipperSubsystem));
    new JoystickButton(Joystick1, 5).onTrue(new FlipperGripperCmd(m_flipperSubsystem));
    new JoystickButton(Joystick1, 3).onTrue(new ElevatorCommandGroup(m_elevatorSubsystem, 0));
    new JoystickButton(Joystick1, 4).onTrue(new ElevatorCommandGroup(m_elevatorSubsystem, 1));
    new JoystickButton(Joystick1, 6).onTrue(new ElevatorCommandGroup(m_elevatorSubsystem, 2));
  }

  //
  //    Drive Commands
  //
  // Default command, normal field-relative drive

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // Utility function to apply deadband
  private double applyDeadband(double value) {
    return (Math.abs(value) > DEADZONE) ? value : 0.0;
  }
}
