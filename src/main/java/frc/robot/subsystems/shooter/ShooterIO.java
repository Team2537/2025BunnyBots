package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // Loading Motor (NEO)
        public double loadingPositionRad = 0.0;
        public double loadingVelocityRadPerSec = 0.0;
        public double loadingAppliedVolts = 0.0;
        public double loadingSupplyCurrentAmps = 0.0;
        public double loadingTempCelcius = 0.0;

        // Shooter Motor Left (Vortex)
        public double shooterLeftPositionRad = 0.0;
        public double shooterLeftVelocityRadPerSec = 0.0;
        public double shooterLeftAppliedVolts = 0.0;
        public double shooterLeftSupplyCurrentAmps = 0.0;
        public double shooterLeftTempCelcius = 0.0;

        // Shooter Motor Right (Vortex)
        public double shooterRightPositionRad = 0.0;
        public double shooterRightVelocityRadPerSec = 0.0;
        public double shooterRightAppliedVolts = 0.0;
        public double shooterRightSupplyCurrentAmps = 0.0;
        public double shooterRightTempCelcius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {
    }

    /** Run the loading motor at the specified voltage. */
    public default void setLoadingVoltage(double volts) {
    }

    /** Run the shooter motors at the specified voltage. */
    public default void setShooterVoltage(double volts) {
    }

    /** Run the shooter motors at the specified velocity (RPM). */
    public default void setShooterVelocity(double rpm) {
    }

    /** Stop all motors. */
    public default void stop() {
    }
}
