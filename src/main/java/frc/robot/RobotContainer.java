package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.InterfaceExecuteMode;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FlipEleSubsystem;
import frc.robot.subsystems.InterfaceSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.GeomUtil;
import java.io.IOException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;

public class RobotContainer {

  //
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
  static final LEDSubsystem leds = new LEDSubsystem();
  private final Alert cameraFailureAlert;
  // Subsystems
  private static DriveSubsystem drive;
  public final InterfaceSubsystem reef;
  // Controller
  private static final CommandXboxController driver = new CommandXboxController(0);
  // Button Box (split between 12x position and 4x level buttons, too big for one HID)
  private final Joystick codriverInterfaceBranch = new Joystick(2);
  private final Joystick codriverInterfaceOther = new Joystick(3);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  public LoggedDashboardChooser<Integer> cageChooser;

  public final LoggedDashboardChooser<String> controlChooser =
      new LoggedDashboardChooser<>("Single Driver?");
  private boolean singleDriver = true;

  public boolean testingmode = false;
  public Vision aprilTagVision;
  public boolean ForceClimberUp = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    controlChooser.addOption("Single Driver", "a");
    controlChooser.addOption("Two Driver", "b");

    drive = configureDrive();
    reef = configureInterface();
    aprilTagVision = configureAprilTagVision();
    configureNamedCommands();

    // Set up continuous banner detection for the elevator/flipper subsystem
    elevator.setDefaultCommand(new FlipperGripperCmd(elevator, false));

    autoChooser = configureAutos();

    cageChooser = new LoggedDashboardChooser<>("Climber Choices", new SendableChooser<>());
    setupCageChooser();

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

  /** sets up named commands for pathplanner to use */
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

  /** returns the now set-up auto selector */
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

  private void configureButtonBindings() {

    // Normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Squeeze + grip
    driver.leftBumper().and(() -> !singleDriver).onTrue(new FlipperGripperCmd(elevator));

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

    // climb
    driver.start().onTrue(new ActivateClimberCommand(climber));

    driver
        .rightTrigger()
        .and(() -> !singleDriver)
        .whileTrue(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.REEF, -1, false, singleDriver)
                .andThen(
                    () -> {})); // When right trigger is pressed, drive to the location selected
    driver
        .rightTrigger()
        .and(() -> !singleDriver)
        .onFalse(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.DISABLE, -1, false, singleDriver));

    driver
        .rightBumper()
        .and(() -> !singleDriver)
        .whileTrue(
            new InterfaceActionCmd(reef, InterfaceExecuteMode.ALGAE, -1, false)
                .andThen(
                    () -> {})); // When right trigger is pressed, drive to the location selected
    driver
        .rightTrigger()
        .and(() -> !singleDriver)
        .onFalse(new InterfaceActionCmd(reef, InterfaceExecuteMode.DISABLE, -1, false));

    /*
        ---- LED TRIGGERS ----
    */

    // flash white for 1s when a coral is obtained
    Command ledCmdWhite =
        Commands.runOnce(() -> leds.setMode(Constants.LEDMode.CORAL_IN, true))
            .andThen(Commands.waitSeconds(1))
            .andThen(Commands.runOnce(leds::unlock));
    ledCmdWhite.addRequirements(leds);
    new Trigger(elevator::hasObtainedCoral).onTrue(ledCmdWhite);

    // flash blue while taking out algae
    Command ledCmdBlue =
        Commands.runOnce(() -> leds.setMode(Constants.LEDMode.ALGAE_OUT, true))
            .andThen(Commands.waitSeconds(1))
            .andThen(Commands.runOnce(leds::unlock));
    ledCmdBlue.addRequirements(leds);
    driver.y().onTrue(ledCmdBlue);

    // flash green while scoring
    Command ledCmdGreen =
        Commands.runOnce(() -> leds.setMode(Constants.LEDMode.CORAL_OUT, true))
            .andThen(Commands.waitSeconds(1))
            .andThen(Commands.runOnce(leds::unlock));
    ledCmdGreen.addRequirements(leds);
    driver
        .rightTrigger()
        .and(() -> singleDriver)
        .or(driver.leftTrigger())
        .or(driver.leftBumper())
        .or(driver.rightBumper())
        .or(driver.a())
        .or(driver.povRight())
        .onTrue(ledCmdGreen);

    // flash rainbow when climbed
    Command ledCmdRainbow =
        Commands.runOnce(() -> leds.setMode(Constants.LEDMode.RAINBOW, true))
            .andThen(Commands.waitSeconds(1))
            .andThen(Commands.runOnce(leds::unlock));
    ledCmdRainbow.addRequirements(leds);
    new Trigger(climber::getClimberUsed).onTrue(ledCmdRainbow);

    Command ledCmdAir = new InstantCommand((() -> leds.setPSI(drive.getPressurePSI())));
    ledCmdAir.addRequirements(leds);
    leds.setDefaultCommand(ledCmdAir);

    /*
       ---- DRIVER CONTROLS ----
    */

    // manual elevator control
    driver
        .povUp()
        .and(() -> singleDriver)
        .onTrue(
            new InstantCommand(
                    () -> {
                      elevator.centererClosePending = false;
                      elevator.centerer.set(false);
                      elevator.setInScoringMode(true);
                    })
                .andThen(Commands.waitSeconds(0.18))
                .andThen(
                    () -> {
                      elevator.inHoldingState = true;
                      elevator.raiseElevator(4);
                    }));
    driver
        .povLeft()
        .and(() -> singleDriver)
        .onTrue(
            new InstantCommand(
                    () -> {
                      elevator.centererClosePending = false;
                      elevator.centerer.set(false);
                      elevator.setInScoringMode(true);
                    })
                .andThen(Commands.waitSeconds(0.18))
                .andThen(
                    () -> {
                      elevator.inHoldingState = true;
                      elevator.raiseElevator(3);
                    }));
    driver
        .povDown()
        .and(() -> singleDriver)
        .onTrue(
            new InstantCommand(
                    () -> {
                      elevator.centererClosePending = false;
                      elevator.centerer.set(false);
                      elevator.setInScoringMode(true);
                    })
                .andThen(Commands.waitSeconds(0.18))
                .andThen(
                    () -> {
                      elevator.inHoldingState = true;
                      elevator.raiseElevator(2);
                    }));

    // right l3
    driver
        .rightTrigger()
        .and(() -> singleDriver)
        .whileTrue(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.REEF, 3, true, singleDriver)
                .andThen(
                    () -> {})); // When right trigger is pressed, drive to the location selected
    driver
        .rightTrigger()
        .and(() -> singleDriver)
        .onFalse(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.DISABLE, -1, true, singleDriver));

    // right l3
    driver
        .rightBumper()
        .and(() -> singleDriver)
        .whileTrue(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.REEF, 3, true, singleDriver)
                .andThen(
                    () -> {})); // When right trigger is pressed, drive to the location selected
    driver
        .rightBumper()
        .and(() -> singleDriver)
        .onFalse(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.DISABLE, -1, true, singleDriver));

    // left l3
    driver
        .leftTrigger()
        .and(() -> singleDriver)
        .whileTrue(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.REEF, 3, false, singleDriver)
                .andThen(
                    () -> {})); // When right trigger is pressed, drive to the location selected
    driver
        .leftTrigger()
        .and(() -> singleDriver)
        .onFalse(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.DISABLE, -1, false, singleDriver));

    // left l4
    driver
        .leftBumper()
        .and(() -> singleDriver)
        .whileTrue(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.REEF, 4, false, singleDriver)
                .andThen(
                    () -> {})); // When right trigger is pressed, drive to the location selected
    driver
        .leftBumper()
        .and(() -> singleDriver)
        .onFalse(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.DISABLE, -1, false, singleDriver));

    // right l4
    driver
        .rightBumper()
        .and(() -> singleDriver)
        .whileTrue(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.REEF, 4, true, singleDriver)
                .andThen(
                    () -> {})); // When right trigger is pressed, drive to the location selected
    driver
        .rightBumper()
        .and(() -> singleDriver)
        .onFalse(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.DISABLE, -1, true, singleDriver));

    // left l2
    driver
        .povRight()
        .and(() -> singleDriver)
        .whileTrue(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.REEF, 2, false, singleDriver)
                .andThen(
                    () -> {})); // When right trigger is pressed, drive to the location selected
    driver
        .povRight()
        .and(() -> singleDriver)
        .onFalse(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.DISABLE, -1, false, singleDriver));
    // right l2
    driver
        .a()
        .and(() -> singleDriver)
        .whileTrue(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.REEF, 2, true, singleDriver)
                .andThen(
                    () -> {})); // When right trigger is pressed, drive to the location selected
    driver
        .a()
        .and(() -> singleDriver)
        .onFalse(
            new InterfaceActionCmd2(reef, InterfaceExecuteMode.DISABLE, -1, false, singleDriver));

    // algae removal
    driver
        .y()
        .and(() -> singleDriver)
        .whileTrue(
            new InterfaceActionCmd(reef, InterfaceExecuteMode.ALGAE, -1, false)
                .andThen(
                    () -> {})); // When right trigger is pressed, drive to the location selected
    driver
        .y()
        .and(() -> singleDriver)
        .onFalse(new InterfaceActionCmd(reef, InterfaceExecuteMode.DISABLE, -1, false));

    // manual drop coral
    driver.b().onTrue(new FlipperScoreCmd(elevator, 1.0, reef));

    // un-grip
    driver
        .x()
        .onTrue(
            new GripperToggleCmd(
                elevator, true)); // Use B+LB to toggle gripper and reset holding state

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

  private void setupCageChooser() {
    cageChooser.addDefaultOption("Outer Cage", 0);
    cageChooser.addOption("Middle Cage", 1);
    cageChooser.addOption("Inner Cage", 2);
  }

  public void updateCageFromChooser() {
    int selectedCage = cageChooser.get();
    new CageSelectCmd(selectedCage).schedule();
  }

  public static CommandXboxController getController() {
    return driver;
  }

  public void updateControlSchemeFromChooser() {
    String chosen = controlChooser.get();
    if (chosen != null) {
      singleDriver = chosen.equals("a");
    }
  }

  public static LEDSubsystem getLEDSubsystem() {
    return leds;
  }

  public static double getPressureProxy() {
    return drive.getPressurePSI();
  }
}
