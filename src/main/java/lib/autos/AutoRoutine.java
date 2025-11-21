package lib.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.funnel.Funnel;

import java.util.List;

/** Describes an autonomous routine built from branch actions. */
public final class AutoRoutine {
  public enum AutoAction {
    DRIVE_TO_COSMIC_CONVERTER,
    DRIVE_TO_OUT,
    SCORE_LOW,
    SCORE_HIGH,
  }
  private final List<AutoAction> actions;
  private final Drive drive;
  private final Shooter shooter;
  private final Funnel funnel;

  public AutoRoutine(List<AutoAction> actions, Drive drive, Shooter shooter, Funnel funnel) {
    this.actions = actions;
    this.drive = drive;
    this.shooter = shooter;
    this.funnel = funnel;
  }

  public Command build() {
    SequentialCommandGroup sequence = new SequentialCommandGroup();


    AutoAction previousAction = null;

    for (AutoAction action : actions) {
      switch (action) {
        case DRIVE_TO_COSMIC_CONVERTER:
          previousAction = action;
          sequence.addCommands(
            Commands.sequence(
              AutoBuilder.resetOdom(getPathFromStartToCosmicConverter().getStartingHolonomicPose().orElseGet(Pose2d::new)),
              AutoBuilder.followPath(getPathFromStartToCosmicConverter()).andThen(Commands.runOnce(drive::stopWithX, drive))
            )
          );
          break;
        case DRIVE_TO_OUT:
          if (previousAction == AutoAction.DRIVE_TO_COSMIC_CONVERTER) {
            previousAction = null;
            sequence.addCommands(
              Commands.sequence(
                AutoBuilder.followPath(getPathFromCosmicConverterToOut()).andThen(Commands.runOnce(drive::stopWithX, drive))
              )
            );
          } else {
            sequence.addCommands(
              Commands.sequence(
                AutoBuilder.resetOdom(getPathFromStartToOut().getStartingHolonomicPose().orElseGet(Pose2d::new)),
                AutoBuilder.followPath(getPathFromStartToOut()).andThen(Commands.runOnce(drive::stopWithX, drive))
              )
            );
          }
          break;
        case SCORE_LOW:
          sequence.addCommands(
            Commands.deadline(
              Commands.waitSeconds(5),
              funnel.runFunnel(),
              shooter.shootLow()
            )
          );
          break;
        case SCORE_HIGH:
          sequence.addCommands(
            Commands.deadline(
              Commands.waitSeconds(5),
              funnel.runFunnel(),
              shooter.shootHigh()
            )
          );
          break;
      }
    }

    return sequence;
  }

  private PathPlannerPath getPathFromStartToCosmicConverter() {
    try {
      return PathPlannerPath.fromPathFile("start_to_cosmic_converter");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load path file", e);
    }
  }

  private PathPlannerPath getPathFromCosmicConverterToOut() {
    try {
      return PathPlannerPath.fromPathFile("cosmic_converter_to_out");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load path file", e);
    }
  }

  private PathPlannerPath getPathFromStartToOut() {
    try {
      return PathPlannerPath.fromPathFile("start_to_out");
    } catch (Exception e) {
      throw new RuntimeException("Failed to load path file", e);
    }
  }
}
