package frc.robot.subsystems.funnel;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.Units;

import frc.robot.Constants.FunnelConstants;

public class FunnelIOReal implements FunnelIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public FunnelIOReal() {
        motor = new SparkMax(FunnelConstants.MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit((int) FunnelConstants.CURRENT_LIMIT.in(Units.Amps));
        config.inverted(FunnelConstants.INVERTED);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(FunnelIOInputs inputs) {
        inputs.positionRad = Units.Rotations.of(encoder.getPosition()).in(Units.Radians);
        inputs.velocityRadPerSec = Units.RadiansPerSecond.of(encoder.getVelocity()).in(Units.RadiansPerSecond);
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.supplyCurrentAmps = motor.getOutputCurrent();
        inputs.tempCelcius = motor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
