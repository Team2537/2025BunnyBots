package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.AlignmentState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.funnel.Funnel;
import java.util.List;
import java.util.function.Supplier;
import lib.autos.AutoRoutine;
import lib.autos.AutoRoutine.AutoAction;
import lib.math.geometry.FieldConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Collection of autonomous routine factories. */
public final class Autos {
        private final Drive drive;
        private final AlignmentState alignmentState;
        private final Shooter shooter;
        private final Funnel funnel;
        private final LoggedDashboardChooser<Supplier<Command>> chooser = new LoggedDashboardChooser<>("auto");

        private final AutoRoutine idleRoutine;
        private final AutoRoutine exitRoutine;
        private final AutoRoutine scoreHighRoutine;
        private final AutoRoutine scoreLowRoutine;

        public Autos(Drive drive, AlignmentState alignmentState, Shooter shooter, Funnel funnel) {
                this.drive = drive;
                this.alignmentState = alignmentState;
                this.shooter = shooter;
                this.funnel = funnel;
                this.exitRoutine = new AutoRoutine(List.of(
                                AutoAction.DRIVE_TO_OUT), drive, shooter, funnel);
                this.scoreHighRoutine = new AutoRoutine(List.of(
                                AutoAction.DRIVE_TO_COSMIC_CONVERTER,
                                AutoAction.SCORE_HIGH,
                                AutoAction.DRIVE_TO_OUT), drive, shooter, funnel);
                this.scoreLowRoutine = new AutoRoutine(List.of(
                                AutoAction.DRIVE_TO_COSMIC_CONVERTER,
                                AutoAction.SCORE_LOW,
                                AutoAction.DRIVE_TO_OUT), drive, shooter, funnel);
                this.idleRoutine = new AutoRoutine(List.of(), drive, shooter, funnel);

                // chooser.addOption("idle", idleRoutine::build);
                // chooser.addOption("scoreHigh", scoreHighRoutine::build);
                // chooser.addOption("scoreLow", scoreLowRoutine::build);
                // chooser.addDefaultOption("exit", exitRoutine::build);

                chooser.addOption("idle", idleRoutine::build);
                chooser.addDefaultOption("scoreHigh", scoreHighRoutine::build);
                chooser.addOption("scoreLow", scoreLowRoutine::build);
                chooser.addOption("exit", exitRoutine::build);

                SmartDashboard.putData(chooser.getSendableChooser());
        }

        public Command getSelectedRoutine() {
                return chooser.get().get();
        }
}
