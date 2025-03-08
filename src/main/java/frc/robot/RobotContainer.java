package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.InterfaceExecuteMode;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.Vision;
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
  public final PhotonCamera frontLeftCamera = new PhotonCamera("front-left");
  public final PhotonCamera frontRightCamera = new PhotonCamera("front-right");
  public final PhotonCamera backLeftCamera = new PhotonCamera("back_left");
  public final PhotonCamera backRightCamera = new PhotonCamera("back_right");
  public final PhotonCamera frontCenterCamera = new PhotonCamera("front-center");

  private final Alert cameraFailureAlert;

  // Subsystems
  private final DriveSubsystem drive;
  private InterfaceSubsystem reef;
  private final FlipperSubsystem flipper = new FlipperSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final PressureSubsystem pressure = new PressureSubsystem();

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  private final Joystick cojoystick = new Joystick(1);
  // There are two codriver joystick ports because only 12 buttons can be detected, and just the
  // branch select is 12 buttons.
  //   private final Joystick codriverInterfaceBranch = new Joystick(2);
  //   private final Joystick codriverInterfaceOther = new Joystick(3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  public Vision aprilTagVision;
  // Create the target Transform2d (Translation and Rotation)
  //   Translation2d targetTranslation = new Translation2d(13.8, 5.6); // X = 14, Y = 4
  //   Rotation2d targetRotation = new Rotation2d(Units.degreesToRadians(-121.0)); // No rotation
  Translation2d targetTranslation = new Translation2d(12.225, 2.474); // X = 14, Y = 4
  Rotation2d targetRotation = new Rotation2d(Units.degreesToRadians(60.0)); // No rotation

  // new Pose2d(13.714, 5.136, Rotation2d.fromDegrees(-120.000));
  //   Translation2d targetTranslation = new Translation2d(13.5, 5.5); // DO NOT TOUCH
  //   Translation2d targetTranslation = new Translation2d(14.186, 5.136); // for later
  //   Rotation2d targetRotation = new Rotation2d(Units.degreesToRadians(-120.0)); //

  Transform2d targetTransform = new Transform2d(targetTranslation, targetRotation);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    reef = configureInterface();
    drive = configureDrive();
    autoChooser = configureAutos();
    aprilTagVision = configureAprilTagVision();
    configureButtonBindings();
    cameraFailureAlert = new Alert("Camera failure.", Alert.AlertType.kError);
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
      aprilTagVision = new Vision(frontLeftCamera, frontRightCamera, frontCenterCamera);

      //   backLeftCamera,
      //   backRightCamera,
      //   frontCenterCamera
    } catch (IOException e) {
      assert cameraFailureAlert != null;
      cameraFailureAlert.set(true);
    }
    aprilTagVision.setDataInterfaces(drive::getPose, drive::addAutoVisionMeasurement);
    return aprilTagVision;
  }

  private InterfaceSubsystem configureInterface() {
    return new InterfaceSubsystem(drive, flipper, elevator);
  }

  private DriveSubsystem configureDrive() {
    // Real robot, instantiate hardware IO implementations
    // Sim robot, instantiate physics sim IO implementations
    // Replayed robot, disable IO implementations
    return switch (Constants.CURRENT_MODE) {
      case REAL ->
          // Real robot, instantiate hardware IO implementations
          new DriveSubsystem(
              new GyroIOPigeon2(),
              new ModuleIOTalonFX(TunerConstants.FrontLeft),
              new ModuleIOTalonFX(TunerConstants.FrontRight),
              new ModuleIOTalonFX(TunerConstants.BackLeft),
              new ModuleIOTalonFX(TunerConstants.BackRight));
      case SIM ->
          // Sim robot, instantiate physics sim IO implementations
          new DriveSubsystem(
              new GyroIO() {},
              new ModuleIOSim(TunerConstants.FrontLeft),
              new ModuleIOSim(TunerConstants.FrontRight),
              new ModuleIOSim(TunerConstants.BackLeft),
              new ModuleIOSim(TunerConstants.BackRight));
      default ->
          // Replayed robot, disable IO implementations
          new DriveSubsystem(
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
    // driver
    // .a()
    // .whileTrue(
    //     DriveCommands.joystickDriveAtAngle(
    //         drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), Rotation2d::new));

    // Switch to X pattern when X button is pressed
    // driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // need both commands
    driver // Right reef facing red riverstation (temporary)
        .rightTrigger()
        .whileTrue(
            DriveCommands.goToTransformWithPathFinderPlusOffset(
                drive, targetTransform, new Transform2d(0.50, -0.24, new Rotation2d())));
    // DriveCommands.goToTransformWithPathFinder(drive, targetTransform)
    //     .andThen(
    //         DriveCommands.goToTransform(
    //             drive,
    //             targetTransform.plus(new Transform2d(0.50, -0.23, new Rotation2d()))))
    //     .beforeStarting(
    //         () -> {
    //           DriveCommands.goToTransform(drive, targetTransform).cancel();
    //           DriveCommands.goToTransformWithPathFinder(drive, targetTransform).cancel();
    //         }));

    driver // Left reef facing red driverstatoin (temporary)
        .leftTrigger()
        .whileTrue(
            DriveCommands.goToTransformWithPathFinderPlusOffset(
                drive,
                targetTransform,
                new Transform2d(0.50, 0.11, new Rotation2d(Units.degreesToRadians(0.0)))));

    driver // Left reef facing red driverstatoin (temporary)
        .rightBumper()
        .whileTrue(
            DriveCommands.goToTransformWithPathFinderPlusOffset(
                    drive,
                    targetTransform,
                    new Transform2d(0.52, -0.05, new Rotation2d(Units.degreesToRadians(2.0))))
                .andThen(new WaitForTimeCmd(0.5))
                .andThen(new ElevatorCmd(elevator, 2, true))
                .andThen(new WaitForTimeCmd(0.5))
                .andThen(new FlipperScoreCmd(flipper, 10.0))
                .andThen(new WaitForTimeCmd(0.5))
                .andThen(new ElevatorCmd(elevator, 2, false))
                .andThen(new WaitForTimeCmd(1.1))
                .andThen(DriveCommands.goToTransform(drive, targetTransform)));
    // DriveCommands.goToTransformWithPathFinder(drive, targetTransform)
    //     .andThen(
    //         DriveCommands.goToTransform(
    //             drive,
    //             targetTransform.plus(
    //                 new Transform2d(
    //                     0.51, 0.14, new Rotation2d(Units.degreesToRadians(-2.0))))))
    //     .beforeStarting(
    //         () -> {
    //           DriveCommands.goToTransform(drive, targetTransform).cancel();
    //           DriveCommands.goToTransformWithPathFinder(drive, targetTransform).cancel();
    //         }));

    driver
        .leftBumper()
        .whileTrue(
            DriveCommands.goToTransformWithPathFinder(
                    drive,
                    new Transform2d(16.7, 1.2, new Rotation2d(Units.degreesToRadians(-45.0))))
                .andThen(
                    DriveCommands.goToTransform(
                        drive,
                        new Transform2d(16.7, 1.2, new Rotation2d(Units.degreesToRadians(-45.0)))
                            .plus(new Transform2d(0.15, 0.15, new Rotation2d()))))
                .beforeStarting(
                    () -> {
                      DriveCommands.goToTransform(drive, targetTransform).cancel();
                      DriveCommands.goToTransformWithPathFinder(drive, targetTransform).cancel();
                    }));

    // Reset gyro to 0 when B button is pressed
    // driver
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    //
    //    Codriver Bindings
    //
    // Reef branch selection
    // new JoystickButton(codriverInterfaceBranch, 1)
    //     .onTrue(new InterfaceVarsCmd(reef, "a", 0, true, false));
    // new JoystickButton(codriverInterfaceBranch, 2)
    //     .onTrue(new InterfaceVarsCmd(reef, "b", 0, true, false));
    // new JoystickButton(codriverInterfaceBranch, 3)
    //     .onTrue(new InterfaceVarsCmd(reef, "c", 0, true, false));
    // new JoystickButton(codriverInterfaceBranch, 4)
    //     .onTrue(new InterfaceVarsCmd(reef, "d", 0, true, false));
    // new JoystickButton(codriverInterfaceBranch, 5)
    //     .onTrue(new InterfaceVarsCmd(reef, "e", 0, true, false));
    // new JoystickButton(codriverInterfaceBranch, 6)
    //     .onTrue(new InterfaceVarsCmd(reef, "f", 0, true, false));
    // new JoystickButton(codriverInterfaceBranch, 7)
    //     .onTrue(new InterfaceVarsCmd(reef, "g", 0, true, false));
    // new JoystickButton(codriverInterfaceBranch, 8)
    //     .onTrue(new InterfaceVarsCmd(reef, "h", 0, true, false));
    // new JoystickButton(codriverInterfaceBranch, 9)
    //     .onTrue(new InterfaceVarsCmd(reef, "i", 0, true, false));
    // new JoystickButton(codriverInterfaceBranch, 10)
    //     .onTrue(new InterfaceVarsCmd(reef, "j", 0, true, false));
    // new JoystickButton(codriverInterfaceBranch, 11)
    //     .onTrue(new InterfaceVarsCmd(reef, "k", 0, true, false));
    // new JoystickButton(codriverInterfaceBranch, 12)
    //     .onTrue(new InterfaceVarsCmd(reef, "l", 0, true, false));

    // // Climber toggle, elevator level selection
    // new JoystickButton(codriverInterfaceOther, 1)
    //     .and(new JoystickButton(codriverInterfaceOther, 2))
    //     .onTrue(new ActivateClimberCommand(climber));
    // new JoystickButton(codriverInterfaceOther, 3)
    //     .onTrue(new InterfaceVarsCmd(reef, "", 1, false, true));
    // new JoystickButton(codriverInterfaceOther, 4)
    //     .onTrue(new InterfaceVarsCmd(reef, "", 2, false, true));
    // new JoystickButton(codriverInterfaceOther, 5)
    //     .onTrue(new InterfaceVarsCmd(reef, "", 3, false, true));
    // new JoystickButton(codriverInterfaceOther, 6)
    //     .onTrue(new InterfaceVarsCmd(reef, "", 4, false, true));

    driver.a().onTrue(new InterfaceActionCmd(reef, InterfaceExecuteMode.EXECUTE));
    driver
        .rightTrigger()
        .onTrue(
            new InterfaceActionCmd(
                reef,
                InterfaceExecuteMode
                    .REEF)); // When right trigger is pressed, drive to the location selected
    driver.x().onTrue(new InterfaceActionCmd(reef, InterfaceExecuteMode.CORAL));
    driver.y().onTrue(new InterfaceActionCmd(reef, InterfaceExecuteMode.CLIMBER));
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
