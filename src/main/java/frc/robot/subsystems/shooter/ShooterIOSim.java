package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

/** Physics-based simulation for the shooter and loader motors. */
public class ShooterIOSim implements ShooterIO {
    private static final double LOOP_PERIOD_SEC = Robot.getUpdateRateSec();
    private static final double LOADER_MOI_KG_M2 = 0.004;
    private static final double SHOOTER_MOI_KG_M2 = 0.002;
    private static final double GEAR_RATIO = 1.0;
    private static final double MAX_RPM = 6000.0;
    private static final double MAX_RAD_PER_SEC = MAX_RPM * 2.0 * Math.PI / 60.0;

    private final DCMotorSim loadingSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getNEO(1), LOADER_MOI_KG_M2, GEAR_RATIO),
            DCMotor.getNEO(1));
    private final DCMotorSim shooterLeftSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(1), SHOOTER_MOI_KG_M2, GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1));
    private final DCMotorSim shooterRightSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(1), SHOOTER_MOI_KG_M2, GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1));

    private final PIDController shooterVelocityController = new PIDController(0.01, 0.0, 0.0);
    private final SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(
            0.05,
            12.0 / MAX_RAD_PER_SEC,
            0.002);

    private double loadingAppliedVolts = 0.0;
    private double shooterAppliedVolts = 0.0;
    private double shooterVelocitySetpointRadPerSec = 0.0;
    private boolean shooterClosedLoop = false;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        if (shooterClosedLoop) {
            double averageVelocityRadPerSec = (shooterLeftSim.getAngularVelocityRadPerSec()
                    + shooterRightSim.getAngularVelocityRadPerSec()) / 2.0;
            double ffVolts = shooterFeedforward.calculate(shooterVelocitySetpointRadPerSec);
            double fbVolts = shooterVelocityController.calculate(
                    averageVelocityRadPerSec,
                    shooterVelocitySetpointRadPerSec);
            shooterAppliedVolts = MathUtil.clamp(ffVolts + fbVolts, -12.0, 12.0);
        } else {
            shooterVelocityController.reset();
        }

        loadingSim.setInputVoltage(MathUtil.clamp(loadingAppliedVolts, -12.0, 12.0));
        shooterLeftSim.setInputVoltage(shooterAppliedVolts);
        shooterRightSim.setInputVoltage(shooterAppliedVolts);

        loadingSim.update(LOOP_PERIOD_SEC);
        shooterLeftSim.update(LOOP_PERIOD_SEC);
        shooterRightSim.update(LOOP_PERIOD_SEC);

        inputs.loadingPositionRad = loadingSim.getAngularPositionRad();
        inputs.loadingVelocityRadPerSec = loadingSim.getAngularVelocityRadPerSec();
        inputs.loadingAppliedVolts = loadingAppliedVolts;
        inputs.loadingSupplyCurrentAmps = Math.abs(loadingSim.getCurrentDrawAmps());
        inputs.loadingTempCelcius = 25.0;

        inputs.shooterLeftPositionRad = shooterLeftSim.getAngularPositionRad();
        inputs.shooterLeftVelocityRadPerSec = shooterLeftSim.getAngularVelocityRadPerSec();
        inputs.shooterLeftAppliedVolts = shooterAppliedVolts;
        inputs.shooterLeftSupplyCurrentAmps = Math.abs(shooterLeftSim.getCurrentDrawAmps());
        inputs.shooterLeftTempCelcius = 30.0;

        inputs.shooterRightPositionRad = shooterRightSim.getAngularPositionRad();
        inputs.shooterRightVelocityRadPerSec = shooterRightSim.getAngularVelocityRadPerSec();
        inputs.shooterRightAppliedVolts = shooterAppliedVolts;
        inputs.shooterRightSupplyCurrentAmps = Math.abs(shooterRightSim.getCurrentDrawAmps());
        inputs.shooterRightTempCelcius = 30.0;
    }

    @Override
    public void setLoadingVoltage(double volts) {
        loadingAppliedVolts = volts;
    }

    @Override
    public void setShooterVoltage(double volts) {
        shooterClosedLoop = false;
        shooterAppliedVolts = volts;
    }

    @Override
    public void setShooterVelocity(double rpm) {
        shooterClosedLoop = true;
        shooterVelocitySetpointRadPerSec = rpm * 2.0 * Math.PI / 60.0;
    }

    @Override
    public void stop() {
        loadingAppliedVolts = 0.0;
        shooterAppliedVolts = 0.0;
        shooterClosedLoop = false;
    }
}
