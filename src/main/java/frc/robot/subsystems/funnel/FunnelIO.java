package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
    @AutoLog
    public static class FunnelIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelcius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(FunnelIOInputs inputs) {
    }

    /** Run the motor at the specified voltage. */
    public default void setVoltage(double volts) {
    }

    /** Stop the motor. */
    public default void stop() {
    }
}
