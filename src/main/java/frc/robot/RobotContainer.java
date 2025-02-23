package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.LedMode;
import frc.robot.commandgroups.ElevatorCommandGroup;
import frc.robot.commands.ChooseReefCmd;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FlipperGripperCmd;
import frc.robot.commands.FlipperScoreCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlipperSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PhysicalReefInterfaceSubsystem;
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
  public final PhotonCamera frontCenterCamera = new PhotonCamera("front_center");

  private final Alert cameraFailureAlert;

  // Subsystems
  private final DriveSubsystem drive;
  private final FlipperSubsystem flipper = new FlipperSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final LedSubsystem led = new LedSubsystem();
  private final PhysicalReefInterfaceSubsystem reef = new PhysicalReefInterfaceSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  private final Joystick cojoystick = new Joystick(1);
  private final Joystick reefController = new Joystick(2);

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
      aprilTagVision =
          new Vision(
              frontLeftCamera,
              frontRightCamera,
              backLeftCamera,
              backRightCamera,
              frontCenterCamera);
    } catch (IOException e) {
      assert cameraFailureAlert != null;
      cameraFailureAlert.set(true);
    }
    aprilTagVision.setDataInterfaces(drive::getPose, drive::addAutoVisionMeasurement);
    return aprilTagVision;
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
    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), Rotation2d::new));

    // Switch to X pattern when X button is pressed
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // need both commands
    driver
        .rightBumper()
        .whileTrue(
            DriveCommands.goToTransformWithPathFinder(drive, targetTransform)
                .andThen(DriveCommands.goToTransform(drive, targetTransform))
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
    // execute
    new JoystickButton(reefController, 1).onTrue(new ChooseReefCmd(reef, -1, -1, -1, true));
    // level
    new JoystickButton(reefController, 2).onTrue(new ChooseReefCmd(reef, 0, -1, -1, false));
    new JoystickButton(reefController, 3).onTrue(new ChooseReefCmd(reef, 1, -1, -1, false));
    new JoystickButton(reefController, 4).onTrue(new ChooseReefCmd(reef, 2, -1, -1, false));
    new JoystickButton(reefController, 6).onTrue(new ChooseReefCmd(reef, 3, -1, -1, false));
    // pos
    new JoystickButton(reefController, 7).onTrue(new ChooseReefCmd(reef, -1, 0, -1, false));
    new JoystickButton(reefController, 8).onTrue(new ChooseReefCmd(reef, -1, 1, -1, false));
    new JoystickButton(reefController, 9).onTrue(new ChooseReefCmd(reef, -1, 2, -1, false));
    new JoystickButton(reefController, 10).onTrue(new ChooseReefCmd(reef, -1, 3, -1, false));
    new JoystickButton(reefController, 11).onTrue(new ChooseReefCmd(reef, -1, 4, -1, false));
    new JoystickButton(reefController, 12).onTrue(new ChooseReefCmd(reef, -1, 5, -1, false));
    // rightleft
    new JoystickButton(reefController, 5).onTrue(new ChooseReefCmd(reef, -1, -1, 1, false));

    //
    //    Standard Joystick Bindings
    // not sure if these should be cojoystick
    //
    new JoystickButton(cojoystick, 1).onTrue(new FlipperScoreCmd(flipper));
    new JoystickButton(cojoystick, 5).onTrue(new FlipperGripperCmd(flipper));
    new JoystickButton(cojoystick, 3).onTrue(new ElevatorCommandGroup(elevator, 0));
    new JoystickButton(cojoystick, 4).onTrue(new ElevatorCommandGroup(elevator, 1));
    new JoystickButton(cojoystick, 6).onTrue(new ElevatorCommandGroup(elevator, 2));

    // LED Triggers

    // Coral
    Trigger ledComplexTrigger = new Trigger(flipper::getHasCoral);
    ledComplexTrigger.onTrue(
        new InstantCommand(
            () -> {
              led.setMode(LedMode.STROBE);
              led.setStrobeSetting(0);
            }));
    ledComplexTrigger.onFalse(new InstantCommand(() -> led.setMode(LedMode.SOLID)));

    // Climbing
    Trigger ledClimbingTrigger = new Trigger(climber::getClimberUsed);
    ledClimbingTrigger.onTrue(
        new InstantCommand(
            () -> {
              led.setMode(LedMode.STROBE);
              led.setStrobeSetting(1);
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
