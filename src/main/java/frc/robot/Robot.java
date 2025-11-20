package frc.robot;

import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.AlignmentState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOHybridFXS;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelIO;
import frc.robot.subsystems.funnel.FunnelIOReal;
import frc.robot.subsystems.funnel.FunnelIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.generated.TunerConstants;
import lib.controllers.CommandButtonBoard;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public final class Robot extends LoggedRobot {
  public static final double UPDATE_RATE_SECONDS = 0.02;

  private static Drive drive;
  private static Funnel funnel;
  private static Shooter shooter;
  private static AlignmentState alignmentState;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandButtonBoard operatorController = new CommandButtonBoard(1, 2);
  private final CommandXboxController godController = new CommandXboxController(5);

  private Autos autos;

  public Robot() {
    HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Java, 0, WPILibVersion.Version);

    Logger.recordMetadata("Type", RobotType.TYPE.toString());
    Logger.recordMetadata("Serial Number", HALUtil.getSerialNumber());
    Logger.recordOutput("Git Dirty", BuildConstants.DIRTY == 1 ? "DIRTY" : "CLEAN");
    Logger.recordOutput("Git Branch", BuildConstants.GIT_BRANCH);
    Logger.recordOutput("Git SHA", BuildConstants.GIT_SHA);
    Logger.recordOutput("Git Date", BuildConstants.GIT_DATE);

    switch (RobotType.MODE) {
      case REAL -> {
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter());
        new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
      }
      case SIMULATION -> {
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter());
      }
      case REPLAY -> {
        setUseTiming(false);
        String logFile = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logFile));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logFile, "_replayed")));
      }
    }

    Logger.start();

    CanandEventLoop.getInstance();
    // FieldConstants.Reef.getNodes();

    CommandScheduler.getInstance()
        .onCommandInitialize(command -> Logger.recordOutput("commands/" + command.getName(), true));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> Logger.recordOutput("commands/" + command.getName(), false));

    CameraServer.startAutomaticCapture();

    // Initialize drive subsystem
    switch (RobotType.MODE) {
      case REAL ->
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOHybridFXS(TunerConstants.FrontLeft),
            new ModuleIOHybridFXS(TunerConstants.FrontRight),
            new ModuleIOHybridFXS(TunerConstants.BackLeft),
            new ModuleIOHybridFXS(TunerConstants.BackRight));
      case SIMULATION ->
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
      default ->
        drive = new Drive(
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
    }

    // Initialize funnel subsystem
    switch (RobotType.MODE) {
      case REAL -> funnel = new Funnel(new FunnelIOReal());
      case SIMULATION -> funnel = new Funnel(new FunnelIOSim());
      default -> funnel = new Funnel(new FunnelIO() {
      });
    }

    switch (RobotType.MODE) {
      case REAL -> shooter = new Shooter(new ShooterIOReal());
      case SIMULATION -> shooter = new Shooter(new ShooterIOSim());
      default -> shooter = new Shooter(new ShooterIO() {
      });
    }

    alignmentState = new AlignmentState();

    // autos = new Autos(drive, alignmentState);

    configureBindings();
  }

  private void configureBindings() {

    if (RobotType.MODE == RobotType.MODE.REAL) {
      drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, driverController::getLeftY, driverController::getLeftX, () -> -driverController.getRightX()));
    } else {
      drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), () -> -driverController.getRightX()));
    }

    // left bumper will be used to toggle slow mode
    driverController.leftBumper().onTrue(drive.toggleSlowMode());

    driverController.a().whileTrue(shooter.shootHigh());

    driverController.b().whileTrue(shooter.shootLow());

    driverController.x().whileTrue(funnel.runFunnel());

    driverController
        .leftStick()
        .onTrue(DriveCommands.toggleFieldOriented(drive));

    driverController
        .povDown()
        .onTrue(DriveCommands.resetOdometryAndHeading(drive));

    driverController.start().onTrue(DriveCommands.feedforwardCharacterization(drive));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    autos.getSelectedRoutine().schedule();
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void simulationPeriodic() {
    MechanismVisualizer.updatePoses();
  }

  public static double getUpdateRateSec() {
    return UPDATE_RATE_SECONDS;
  }

  public static Drive getDrive() {
    return drive;
  }

  public static AlignmentState getAlignmentState() {
    return alignmentState;
  }
}
