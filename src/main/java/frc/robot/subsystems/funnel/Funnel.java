package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FunnelConstants;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
    private final FunnelIO io;
    private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();

    public Funnel(FunnelIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Funnel", inputs);
    }

    public void stop() {
        io.stop();
    }

    /**
     * Returns a command that runs the funnel at a constant speed.
     * Can be used with .toggleOnTrue() for toggle behavior.
     */
    public Command runFunnel() {
        return startEnd(
                () -> io.setVoltage(FunnelConstants.FUNNEL_SPEED.in(edu.wpi.first.units.Units.Volts)),
                this::stop);
    }

    public Command reverseFunnel() {
        return startEnd(
                () -> io.setVoltage(FunnelConstants.REVERSE_FUNNEL_SPEED.in(edu.wpi.first.units.Units.Volts)),
                this::stop);
    }
}
