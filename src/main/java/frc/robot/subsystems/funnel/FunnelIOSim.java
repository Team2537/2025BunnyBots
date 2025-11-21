package frc.robot.subsystems.funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

/** Simple physics-based simulation for the funnel motor. */
public class FunnelIOSim implements FunnelIO {
    private static final double LOOP_PERIOD_SEC = Robot.getUpdateRateSec();
    private static final DCMotor GEARBOX = DCMotor.getNEO(1);
    private static final double GEAR_RATIO = 1.0;
    private static final double MOMENT_OF_INERTIA_KG_M2 = 0.003;

    private final DCMotorSim funnelSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(GEARBOX, MOMENT_OF_INERTIA_KG_M2, GEAR_RATIO),
            GEARBOX);

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(FunnelIOInputs inputs) {
        funnelSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
        funnelSim.update(LOOP_PERIOD_SEC);

        inputs.positionRad = funnelSim.getAngularPositionRad();
        inputs.velocityRadPerSec = funnelSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.supplyCurrentAmps = Math.abs(funnelSim.getCurrentDrawAmps());
        inputs.tempCelcius = 25.0;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
    }

    @Override
    public void stop() {
        appliedVolts = 0.0;
    }
}
