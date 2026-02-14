package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Robot-wide constants expressed in standard units. */
public final class Constants {
  private Constants() {
  }

  public static final class FunnelConstants {
    public static final int MOTOR_ID = 15;
    public static final boolean INVERTED = false;
    public static final Voltage FUNNEL_SPEED = Units.Volts.of(12.0);
    public static final Voltage REVERSE_FUNNEL_SPEED = Units.Volts.of(-12.0);
    public static final Current CURRENT_LIMIT = Units.Amps.of(80.0);

    private FunnelConstants() {
    }
  }

  public static final class ShooterConstants {
    public static final int LOADING_MOTOR_ID = 22;
    public static final int SHOOTER_MOTOR_TOP_ID = 31;
    public static final int SHOOTER_MOTOR_BOTTOM_ID = 23;

    public static final boolean LOADING_INVERTED = true;
    public static final boolean SHOOTER_TOP_INVERTED = false;
    public static final boolean SHOOTER_BOTTOM_INVERTED = false; // Usually one is inverted

    public static final Current LOADING_CURRENT_LIMIT = Units.Amps.of(30.0);
    public static final Current SHOOTER_CURRENT_LIMIT = Units.Amps.of(60.0); // Vortex can handle more

    public static final double SHOOTER_KP = 0.0001;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.0;
    public static final double SHOOTER_KFF = 0.000156;

    public static final double SHOOTER_SPEED_HIGH_RPM = 4500.0;
    public static final double SHOOTER_SPEED_LOW_RPM = 1500.0;
    public static final double SHOOTER_SPEED_REVERSE = -1500.0;
    public static final double SHOOTER_TOLERANCE_RPM = 100.0;

    public static final double LOADER_SPEED_VOLTS = 10.0; // Or use percent/volts
    public static final double LOADER_SPEED_REVERSE_VOLTS = -10.0;

    private ShooterConstants() {
    }
  }

}
