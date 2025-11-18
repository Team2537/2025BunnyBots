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
    public static final Voltage FUNNEL_SPEED = Units.Volts.of(6.0);
    public static final Current CURRENT_LIMIT = Units.Amps.of(40.0);

    private FunnelConstants() {
    }
  }

  public static final class ShooterConstants {
    public static final int LOADING_MOTOR_ID = 16;
    public static final int SHOOTER_MOTOR_LEFT_ID = 17;
    public static final int SHOOTER_MOTOR_RIGHT_ID = 18;

    public static final boolean LOADING_INVERTED = false;
    public static final boolean SHOOTER_LEFT_INVERTED = false;
    public static final boolean SHOOTER_RIGHT_INVERTED = true; // Usually one is inverted

    public static final Current LOADING_CURRENT_LIMIT = Units.Amps.of(30.0);
    public static final Current SHOOTER_CURRENT_LIMIT = Units.Amps.of(60.0); // Vortex can handle more

    public static final double SHOOTER_KP = 0.0001;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.0;
    public static final double SHOOTER_KFF = 0.000156;

    public static final double SHOOTER_SPEED_HIGH_RPM = 5000.0;
    public static final double SHOOTER_SPEED_LOW_RPM = 3000.0;
    public static final double SHOOTER_TOLERANCE_RPM = 50.0;

    public static final double LOADER_SPEED_VOLTS = 8.0; // Or use percent/volts

    private ShooterConstants() {
    }
  }

}
