package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.InterfaceExecuteMode;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FlipEleSubsystem;
import frc.robot.subsystems.InterfaceSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.GeomUtil;
import java.io.IOException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static Pose2d climbTargetTransform =
      new Pose2d(7.736 + 0.15, 7.254 + 0.05, Rotation2d.fromDegrees(-90.0));
  private static Transform2d stationTargetTransform =
      new Transform2d(15.9, 0.72, new Rotation2d(Units.degreesToRadians(-54.4)));
  private static Transform2d stationOffsetTransform =
      new Transform2d(0.2, 0.0, new Rotation2d(0.0));
  public final PhotonCamera frontLeftCamera = new PhotonCamera("front-left");
  public final PhotonCamera frontRightCamera = new PhotonCamera("front-right");
  public final PhotonCamera backLeftCamera = new PhotonCamera("back_left");
  public final PhotonCamera backRightCamera = new PhotonCamera("back_right");
  public final PhotonCamera frontCenterCamera = new PhotonCamera("front-center");
  public final ClimberSubsystem climber = new ClimberSubsystem();
  //   private final PressureSubsystem pressure = new PressureSubsystem();
  final FlipEleSubsystem elevator = new FlipEleSubsystem();
  private final Alert cameraFailureAlert;
  // Subsystems
  private final DriveSubsystem drive;
  private final InterfaceSubsystem reef;
  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final Joystick cojoystick = new Joystick(1);
  // There are two codriver joystick ports because only 12 buttons can be detected, and just the
  // branch select is 12 buttons.
  private final Joystick codriverInterfaceBranch = new Joystick(2);
  private final Joystick codriverInterfaceOther = new Joystick(3);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  public boolean testingmode = false;
  public Vision aprilTagVision;
  public boolean ForceClimberUp = false;
  Translation2d targetTranslation = new Translation2d(12.225, 2.474); // X = 14, Y = 4
  Rotation2d targetRotation = new Rotation2d(Units.degreesToRadians(60.0)); // No rotation
  Transform2d targetTransform = new Transform2d(targetTranslation, targetRotation);
  public static final SendableChooser<Integer> cageChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive = configureDrive();
    reef = configureInterface();
    aprilTagVision = configureAprilTagVision();
    configureNamedCommands();

    setupCageChooser();

    // Set up continuous banner detection for the elevator/flipper subsystem
    elevator.setDefaultCommand(new FlipperGripperCmd(elevator, false));

    autoChooser = configureAutos();
    configureButtonBindings();
    cameraFailureAlert = new Alert("Camera failure.", Alert.AlertType.kError);
  }

  public static void setStationTargetTransform(Transform2d _targetTransform) {
    stationTargetTransform = _targetTransform;
  }

  public static void setStationOffsetTransform(Transform2d _offsetTransform) {
    stationOffsetTransform = _offsetTransform;
  }

  public static void setCageClimb(Pose2d _targetTransform) {
    climbTargetTransform = _targetTransform;
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("grip", new FlipperGripperCmd(elevator));
    NamedCommands.registerCommand("A Branch", new InterfaceVarsCmd(reef, "a", 0, true, false));
    NamedCommands.registerCommand("B Branch", new InterfaceVarsCmd(reef, "b", 0, true, false));
    NamedCommands.registerCommand("C Branch", new InterfaceVarsCmd(reef, "c", 0, true, false));
    NamedCommands.registerCommand("D Branch", new InterfaceVarsCmd(reef, "d", 0, true, false));
    NamedCommands.registerCommand("E Branch", new InterfaceVarsCmd(reef, "e", 0, true, false));
    NamedCommands.registerCommand("F Branch", new InterfaceVarsCmd(reef, "f", 0, true, false));
    NamedCommands.registerCommand("G Branch", new InterfaceVarsCmd(reef, "g", 0, true, false));
    NamedCommands.registerCommand("H Branch", new InterfaceVarsCmd(reef, "h", 0, true, false));
    NamedCommands.registerCommand("I Branch", new InterfaceVarsCmd(reef, "i", 0, true, false));
    NamedCommands.registerCommand("J Branch", new InterfaceVarsCmd(reef, "j", 0, true, false));
    NamedCommands.registerCommand("K Branch", new InterfaceVarsCmd(reef, "k", 0, true, false));
    NamedCommands.registerCommand("L Branch", new InterfaceVarsCmd(reef, "l", 0, true, false));

    NamedCommands.registerCommand("1 Level", new InterfaceVarsCmd(reef, "a", 1, false, true));
    NamedCommands.registerCommand("2 Level", new InterfaceVarsCmd(reef, "a", 2, false, true));
    NamedCommands.registerCommand("3 Level", new InterfaceVarsCmd(reef, "a", 3, false, true));
    NamedCommands.registerCommand("4 Level", new InterfaceVarsCmd(reef, "a", 4, false, true));

    NamedCommands.registerCommand(
        "autoScore", new DeferredCommand(() -> new AutoScoreCommand(reef, drive, elevator)));
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
              backRightCamera,
              frontCenterCamera,
              backLeftCamera);

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
    return new InterfaceSubsystem(drive, elevator);
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
    // Normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Switch to X pattern when X button is pressed
    //    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Squeeze + grip
    //    driver.leftBumper().onTrue(new FlipperGripperCmd(elevator));

    driver.y().onTrue(new CageSelectCmd.CycleCageCmd()); // cycles location of cage

    // Climbing sequence
    driver
        .back()
        .whileTrue(
            new DeferredCommand(
                () ->
                    new InstantCommand(() -> ForceClimberUp = true)
                        .andThen(
                            DriveCommands.goToTransformWithPathFinder(
                                drive, GeomUtil.poseToTransform(climbTargetTransform)))
                        .andThen(
                            DriveCommands.goToTransformClimb(
                                drive,
                                GeomUtil.poseToTransform(climbTargetTransform)
                                    .plus(new Transform2d(0.0, 1.0, new Rotation2d(0.0)))))
                        .andThen(Commands.runOnce(drive::stop, drive))
                        .andThen(Commands.waitSeconds(1.0))
                        .andThen(
                            DriveCommands.goToTransformClimb(
                                drive,
                                GeomUtil.poseToTransform(climbTargetTransform)
                                    .plus(new Transform2d(0.0, 0.8, new Rotation2d(0.0)))))
                        .andThen(Commands.runOnce(drive::stop, drive))
                        .andThen(Commands.waitSeconds(2.0))
                        .andThen(new ActivateClimberCommand(climber))
                        .beforeStarting(
                            () -> {
                              DriveCommands.goToTransformClimb(
                                      drive, GeomUtil.poseToTransform(climbTargetTransform))
                                  .cancel();
                              DriveCommands.goToTransformWithPathFinder(
                                      drive, GeomUtil.poseToTransform(climbTargetTransform))
                                  .cancel();
                            })));

    driver.start().onTrue(new ActivateClimberCommand(climber));
    driver
        .leftTrigger()
        .whileTrue(
            new DeferredCommand(
                () -> {
                  Logger.recordOutput("stationOffset", stationOffsetTransform);
                  return DriveCommands.goToTransformWithPathFinderPlusOffset(
                          drive,
                          stationTargetTransform,
                          new Transform2d(0.25, 0.0, new Rotation2d(0.0)))
                      .beforeStarting(
                          () -> {
                            DriveCommands.goToTransform(drive, stationTargetTransform).cancel();
                            DriveCommands.goToTransformWithPathFinder(drive, stationTargetTransform)
                                .cancel();
                          });
                }));

    // driver.povUp().onTrue(new PickupStationCmd(0)); // Change to upper
    // driver.povDown().onTrue(new PickupStationCmd(1)); // Change to lower
    driver.povLeft().onTrue(new PickupStationCmd(2)); // Change to left
    driver.povRight().onTrue(new PickupStationCmd(3)); // Change to right

    //    driver
    //        .rightTrigger()
    //        .whileTrue(
    //            new InterfaceActionCmd(reef, InterfaceExecuteMode.REEF)
    //                .andThen(
    //                    () -> {})); // When right trigger is pressed, drive to the location
    // selected
    //    driver.rightTrigger().onFalse(new InterfaceActionCmd(reef, InterfaceExecuteMode.DISABLE));

    driver
        .rightTrigger()
        .whileTrue(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.REEF)
                .andThen(
                    () -> {})); // When right trigger is pressed, drive to the location selected
    driver.rightTrigger().onFalse(new InterfaceActionCmd2(reef, InterfaceExecuteMode.DISABLE));

    driver
        .rightBumper()
        .whileTrue(
            new InterfaceActionCmd(reef, InterfaceExecuteMode.ALGEE)
                .andThen(
                    () -> {})); // When right trigger is pressed, drive to the location selected
    driver.rightBumper().onFalse(new InterfaceActionCmd(reef, InterfaceExecuteMode.DISABLE));

    driver.b().onTrue(new FlipperScoreCmd(elevator, 1.0));

    // We need to find an available button for the gripper toggle command
    // Since X is already used for X pattern and we want to keep B for scoring
    // The D-pad is used for pickup station selection
    // Let's use a button chord (simultaneous button press) combination
    driver
        .leftBumper()
        .onTrue(
            new GripperToggleCmd(
                elevator, true)); // Use B+LB to toggle gripper and reset holding state

    // Add elevator toggle button for testing
    driver
        .x()
        .onTrue(
            new ElevatorToggleCmd(elevator)); // Press left stick to toggle elevator between 0 and 2
    //            new ElevatorCmd(elevator, 2, true)
    //                .andThen(new FlipperScoreCmd(flipper, 1.0))
    //                .andThen(
    //                    new ElevatorCmd(
    //                        elevator, 0,
    //                        false))); // TODO: CLEAN UP INTO INTERFACE COMMAND, this is meant to
    // raise
    // elevator to selected level and actuate flipper

    //    Codriver Bindings
    //
    // Reef branch selection
    new JoystickButton(codriverInterfaceBranch, 1)
        .onTrue(new InterfaceVarsCmd(reef, "a", 0, true, false));
    new JoystickButton(codriverInterfaceBranch, 2)
        .onTrue(new InterfaceVarsCmd(reef, "b", 0, true, false));
    new JoystickButton(codriverInterfaceBranch, 3)
        .onTrue(new InterfaceVarsCmd(reef, "c", 0, true, false));
    new JoystickButton(codriverInterfaceBranch, 4)
        .onTrue(new InterfaceVarsCmd(reef, "d", 0, true, false));
    new JoystickButton(codriverInterfaceBranch, 5)
        .onTrue(new InterfaceVarsCmd(reef, "e", 0, true, false));
    new JoystickButton(codriverInterfaceBranch, 6)
        .onTrue(new InterfaceVarsCmd(reef, "f", 0, true, false));
    new JoystickButton(codriverInterfaceBranch, 7)
        .onTrue(new InterfaceVarsCmd(reef, "g", 0, true, false));
    new JoystickButton(codriverInterfaceBranch, 8)
        .onTrue(new InterfaceVarsCmd(reef, "h", 0, true, false));
    new JoystickButton(codriverInterfaceBranch, 9)
        .onTrue(new InterfaceVarsCmd(reef, "i", 0, true, false));
    new JoystickButton(codriverInterfaceBranch, 10)
        .onTrue(new InterfaceVarsCmd(reef, "j", 0, true, false));
    new JoystickButton(codriverInterfaceBranch, 11)
        .onTrue(new InterfaceVarsCmd(reef, "k", 0, true, false));
    new JoystickButton(codriverInterfaceBranch, 12)
        .onTrue(new InterfaceVarsCmd(reef, "l", 0, true, false));

    // Elevator level selection
    new JoystickButton(codriverInterfaceOther, 3)
        .onTrue(new InterfaceVarsCmd(reef, "", 1, false, true));
    new JoystickButton(codriverInterfaceOther, 4)
        .onTrue(new InterfaceVarsCmd(reef, "", 2, false, true));
    new JoystickButton(codriverInterfaceOther, 5)
        .onTrue(new InterfaceVarsCmd(reef, "", 3, false, true));
    new JoystickButton(codriverInterfaceOther, 6)
        .onTrue(new InterfaceVarsCmd(reef, "", 4, false, true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void setTestingModetrue() {
    testingmode = true;
    drive.testingmode = true;
  }

  public static void setupCageChooser() {
    cageChooser.setDefaultOption("Outer Cage", 0);
    cageChooser.addOption("Middle Cage", 1);
    cageChooser.addOption("Inner Cage", 2);
    SmartDashboard.putData("Cage Selector", cageChooser);
  }
  
  public static void updateCageFromChooser() {
    int selectedCage = cageChooser.getSelected();
    new CageSelectCmd(selectedCage).schedule();
  }
  
}
