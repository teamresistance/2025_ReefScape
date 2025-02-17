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

//  ooooooooo.              .o8                     .     .oooooo.                             .              o8o                                 
//  `888   `Y88.           "888                   .o8    d8P'  `Y8b                          .o8              `"'                                 
//   888   .d88'  .ooooo.   888oooo.   .ooooo.  .o888oo 888           .ooooo.  ooo. .oo.   .o888oo  .oooo.   oooo  ooo. .oo.    .ooooo.  oooo d8b 
//   888ooo88P'  d88' `88b  d88' `88b d88' `88b   888   888          d88' `88b `888P"Y88b    888   `P  )88b  `888  `888P"Y88b  d88' `88b `888""8P 
//   888`88b.    888   888  888   888 888   888   888   888          888   888  888   888    888    .oP"888   888   888   888  888ooo888  888     
//   888  `88b.  888   888  888   888 888   888   888 . `88b    ooo  888   888  888   888    888 . d8(  888   888   888   888  888    .o  888     
//  o888o  o888o `Y8bod8P'  `Y8bod8P' `Y8bod8P'   "888"  `Y8bood8P'  `Y8bod8P' o888o o888o   "888" `Y888""8o o888o o888o o888o `Y8bod8P' d888b    

package frc.robot;

// Constants
import frc.robot.Constants.OperatorConstants.JoystickType;
import frc.robot.Constants.HardwareConstants.LedMode;

// Operator Input
import frc.robot.OperatorInput;

//  .oooooo.                                                                             .o8           
// d8P'  `Y8b                                                                           "888           
//888           .ooooo.  ooo. .oo.  .oo.   ooo. .oo.  .oo.    .oooo.   ooo. .oo.    .oooo888   .oooo.o 
//888          d88' `88b `888P"Y88bP"Y88b  `888P"Y88bP"Y88b  `P  )88b  `888P"Y88b  d88' `888  d88(  "8 
//888          888   888  888   888   888   888   888   888   .oP"888   888   888  888   888  `"Y88b.  
//`88b    ooo  888   888  888   888   888   888   888   888  d8(  888   888   888  888   888  o.  )88b 
// `Y8bood8P'  `Y8bod8P' o888o o888o o888o o888o o888o o888o `Y888""8o o888o o888o `Y8bod88P" 8""888P' 

// Command Groups
import frc.robot.commandgroups.Level2CommandGroup;
import frc.robot.commandgroups.Level3CommandGroup;
import frc.robot.commandgroups.Level4CommandGroup;

// Elevator Commands
// NONE YET!

// Arm Commands
// NONE YET!

// Interface Commands
import frc.robot.commands.InterfaceStoreBranchesCommand;
import frc.robot.commands.InterfaceToggleLeftRightCommand;
import frc.robot.commands.InterfaceBranchIDCommand;
import frc.robot.commands.InterfaceBranchLevelCommand;

// Climber Command(s)
import frc.robot.commands.ActivateClimberCommand;

//   .oooooo..o              .o8                                         .                                        
//  d8P'    `Y8             "888                                       .o8                                        
//  Y88bo.      oooo  oooo   888oooo.   .oooo.o oooo    ooo  .oooo.o .o888oo  .ooooo.  ooo. .oo.  .oo.    .oooo.o 
//   `"Y8888o.  `888  `888   d88' `88b d88(  "8  `88.  .8'  d88(  "8   888   d88' `88b `888P"Y88bP"Y88b  d88(  "8 
//       `"Y88b  888   888   888   888 `"Y88b.    `88..8'   `"Y88b.    888   888ooo888  888   888   888  `"Y88b.  
//  oo     .d8P  888   888   888   888 o.  )88b    `888'    o.  )88b   888 . 888    .o  888   888   888  o.  )88b 
//  8""88888P'   `V88V"V8P'  `Y8bod8P' 8""888P'     .8'     8""888P'   "888" `Y8bod8P' o888o o888o o888o 8""888P' 
//                                              .o..P'                                                            
//                                              `Y8P'                                                             

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlipperSubsystem;
import frc.robot.subsystems.InterfaceSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//    .oooooo.   ooooo 
//   d8P'  `Y8b  `888' 
//  888      888  888  
//  888      888  888  
//  888      888  888  
//  `88b    d88'  888  
//   `Y8bood8P'  o888o 
//                    

// Necessary stuff
// import edu.wpi.first.wpilibj2.command.Command;  <------------- Uncomment this if you are using a command without a group
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.Vision;
import java.io.IOException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final PhotonCamera frontLeftCamera = new PhotonCamera("front-left");
  public final PhotonCamera frontRightCamera = new PhotonCamera("front-right");
  public final PhotonCamera backLeftCamera = new PhotonCamera("back_left");
  public final PhotonCamera backRightCamera = new PhotonCamera("back_right");
  public final PhotonCamera frontCenterCamera = new PhotonCamera("front_center");

  // OI
  private final OperatorInput m_operatorInput = new OperatorInput(JoystickType.k3Joysticks);

  // Subsystems
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final FlipperSubsystem m_flipperSubsystem = new FlipperSubsystem();
  private final InterfaceSubsystem m_interfaceSubsystem = new InterfaceSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final LedSubsystem m_ledSubsystem = new LedSubsystem();
  private final Drive drive;

  // Command Groups
  private final Level2CommandGroup m_level2CommandGroup = new Level2CommandGroup(m_flipperSubsystem);
  private final Level3CommandGroup m_level3CommandGroup = new Level3CommandGroup(m_elevatorSubsystem,
      m_flipperSubsystem);
  private final Level4CommandGroup m_level4CommandGroup = new Level4CommandGroup(m_elevatorSubsystem,
      m_flipperSubsystem);
  // Other Commands
  private final InterfaceStoreBranchesCommand m_interfaceStoreBranchesCommand = new InterfaceStoreBranchesCommand(
      m_interfaceSubsystem);
  private final ActivateClimberCommand m_ActivateClimberCommand = new ActivateClimberCommand(m_climberSubsystem);

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController codriver = new CommandXboxController(1);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  public Vision aprilTagVision;
  // Create the target Transform2d (Translation and Rotation)
  Translation2d targetTranslation = new Translation2d(14.26, 4.03); // X = 14, Y = 4
  Rotation2d targetRotation = new Rotation2d(180.0 - 45); // No rotation
  Transform2d targetTransform = new Transform2d(targetTranslation, targetRotation);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drive = configureDrive();
    autoChooser = configureAutos();
    aprilTagVision = configureAprilTagVision();
    configureButtonBindings();
  }

  private LoggedDashboardChooser<Command> configureAutos() {
    // Set up auto routines
    LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices",
        AutoBuilder.buildAutoChooser());

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
      aprilTagVision = new Vision(
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
    return switch (Constants.currentMode) {
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
            new GyroIO() {
            },
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
      default ->
        // Replayed robot, disable IO implementations
        new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
    };
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick}
   * or {@link
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

    driver.leftBumper().whileTrue(DriveCommands.goToTransform(drive, targetTransform));

    driver.rightBumper().whileTrue(DriveCommands.goToTransformWithPathFinder(targetTransform));

    // Reset gyro to 0 when B button is pressed
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive)
                .ignoringDisable(true));

    if (m_operatorInput.joystickType != JoystickType.k2JoysticksAndReefSelector) {
      m_operatorInput.lvl2Button.onTrue(m_level2CommandGroup);
      m_operatorInput.lvl3Button.onTrue(m_level3CommandGroup);
      m_operatorInput.lvl4Button.onTrue(m_level4CommandGroup);
      m_operatorInput.selectBranchAndAddButton.onTrue(m_interfaceStoreBranchesCommand);
    } else {
      m_operatorInput.buttonA.onTrue(
          new InterfaceBranchIDCommand(m_interfaceSubsystem, "A"));
      m_operatorInput.buttonB.onTrue(
          new InterfaceBranchIDCommand(m_interfaceSubsystem, "B"));
      m_operatorInput.buttonC.onTrue(
          new InterfaceBranchIDCommand(m_interfaceSubsystem, "C"));
      m_operatorInput.buttonD.onTrue(
          new InterfaceBranchIDCommand(m_interfaceSubsystem, "D"));
      m_operatorInput.buttonE.onTrue(
          new InterfaceBranchIDCommand(m_interfaceSubsystem, "E"));
      m_operatorInput.buttonF.onTrue(
          new InterfaceBranchIDCommand(m_interfaceSubsystem, "F"));

      m_operatorInput.buttonRL.onTrue(
          new InterfaceToggleLeftRightCommand(m_interfaceSubsystem));

      m_operatorInput.button4.onTrue(
          new InterfaceBranchLevelCommand(m_interfaceSubsystem, 4));
      m_operatorInput.button3.onTrue(
          new InterfaceBranchLevelCommand(m_interfaceSubsystem, 3));
      m_operatorInput.button2_1.onTrue(
          new InterfaceBranchLevelCommand(m_interfaceSubsystem, 2));
    }
    m_operatorInput.climbButton.onTrue(m_ActivateClimberCommand);

    // ------------------Non-button Triggers for Subsystem Interaction

    // If flipper subsystem detects coral, LED subsystem will strobe.
    // If flipper subsystem is ready to score, LED subsystem will glow solid.
    Trigger ledComplexTrigger = new Trigger(() -> m_flipperSubsystem.getHasCoral()
        && (!m_elevatorSubsystem.getFirstStageSolenoidUp()));

    ledComplexTrigger.onTrue(new InstantCommand(
        () -> {
          m_ledSubsystem.setMode(LedMode.kSTROBE);
        }));

    ledComplexTrigger.onFalse(new InstantCommand(
        () -> {
          m_ledSubsystem.setMode(LedMode.kSOLID);
        }));
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
