package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    /**
     * Run the shooter at the specified velocity.
     * 
     * @param rpm Target velocity in RPM
     */
    public void runShooter(double rpm) {
        io.setShooterVelocity(rpm);
    }

    /**
     * Run the loader at the specified voltage.
     * 
     * @param volts Target voltage
     */
    public void runLoader(double volts) {
        io.setLoadingVoltage(volts);
    }

    /** Stop all motors. */
    public void stop() {
        io.stop();
    }

    /**
     * Check if the shooter is at the target speed.
     * 
     * @param targetRpm Target velocity in RPM
     * @return True if at speed
     */
    public boolean atSpeed(double targetRpm) {
        // Check both motors or average?
        // We'll check if both are within tolerance.
        double leftRpm = inputs.shooterLeftVelocityRpm;
        double rightRpm = inputs.shooterRightVelocityRpm;

        return MathUtil.isNear(targetRpm, topRpm, ShooterConstants.SHOOTER_TOLERANCE_RPM) &&
                MathUtil.isNear(targetRpm, bottomRpm, ShooterConstants.SHOOTER_TOLERANCE_RPM);
    }

    /**
     * Command to spin shooter to target speed, then feed.
     * 
     * @param rpm Target shooter speed in RPM
     * @return Command
     */
    public Command shoot(double rpm) {
        return this.runEnd(
                () -> {
                    runShooter(rpm);
                    runLoader(ShooterConstants.LOADER_SPEED_VOLTS);
                },
                () -> stop());
    }

    public Command shootHigh() {
        return shoot(ShooterConstants.SHOOTER_SPEED_HIGH_RPM);
    }

    public Command shootLow() {
        return shoot(ShooterConstants.SHOOTER_SPEED_LOW_RPM);
    }
}
