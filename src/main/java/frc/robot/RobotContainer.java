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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commandgroups.ElevatorHighestCommandGroup;
import frc.robot.commandgroups.ElevatorLowestCommandGroup;
import frc.robot.commandgroups.ElevatorMiddleCommandGroup;
import frc.robot.commandgroups.ScoreCommandGroup;
import frc.robot.commands.ChooseReefCmd;
import frc.robot.commands.FlipperReceivingCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlipperSubsystem;
import frc.robot.subsystems.PhysicalReefInterfaceSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final FlipperSubsystem m_flipperSubsystem = new FlipperSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  final double DEADZONE = 0.1;

  // Create the target Transform2d (Translation and Rotation)
  Translation2d targetTranslation = new Translation2d(15, 4); // X = 14, Y = 4
  Rotation2d targetRotation = new Rotation2d(0.0); // No rotation
  Transform2d targetTransform = new Transform2d(targetTranslation, targetRotation);

  // Controller
  // private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick Joystick1 = new Joystick(0);
  private final Joystick Joystick2 = new Joystick(1);
  private final Joystick CoJoystick = new Joystick(2);

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    //
    //    Experimental Physical Reef Interface
    //
    // Physical reef interface refers to a separate "joystick" which is a few buttons, similar to a
    // macro keyboard
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
    new JoystickButton(Joystick1, 1).onTrue(new ScoreCommandGroup(m_flipperSubsystem));
    new JoystickButton(Joystick1, 5).onTrue(new FlipperReceivingCommand(m_flipperSubsystem));
    new JoystickButton(Joystick1, 3).onTrue(new ElevatorLowestCommandGroup(m_elevatorSubsystem));
    new JoystickButton(Joystick1, 4).onTrue(new ElevatorMiddleCommandGroup(m_elevatorSubsystem));
    new JoystickButton(Joystick1, 6).onTrue(new ElevatorHighestCommandGroup(m_elevatorSubsystem));
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
