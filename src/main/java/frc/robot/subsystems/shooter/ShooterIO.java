package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // Loading Motor (NEO)
        public double loadingPositionRad = 0.0;
        public double loadingVelocityRpm = 0.0;
        public double loadingAppliedVolts = 0.0;
        public double loadingSupplyCurrentAmps = 0.0;
        public double loadingTempCelcius = 0.0;

        // Shooter Motor Top (Vortex)
        public double shooterTopPositionRad = 0.0;
        public double shooterTopVelocityRpm = 0.0;
        public double shooterTopAppliedVolts = 0.0;
        public double shooterTopSupplyCurrentAmps = 0.0;
        public double shooterTopTempCelcius = 0.0;

        // Shooter Motor Bottom (Vortex)
        public double shooterBottomPositionRad = 0.0;
        public double shooterBottomVelocityRpm = 0.0;
        public double shooterBottomAppliedVolts = 0.0;
        public double shooterBottomSupplyCurrentAmps = 0.0;
        public double shooterBottomTempCelcius = 0.0;
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
