package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.Units;

import frc.robot.Constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {
    private final SparkMax loadingMotor;
    private final SparkFlex shooterTopMotor;
    private final SparkFlex shooterBottomMotor;

    private final RelativeEncoder loadingEncoder;
    private final RelativeEncoder shooterTopEncoder;
    private final RelativeEncoder shooterBottomEncoder;

    public ShooterIOReal() {
        // Loading Motor (NEO)
        loadingMotor = new SparkMax(ShooterConstants.LOADING_MOTOR_ID, MotorType.kBrushless);
        loadingEncoder = loadingMotor.getEncoder();

        SparkMaxConfig loadingConfig = new SparkMaxConfig();
        loadingConfig.idleMode(IdleMode.kBrake);
        loadingConfig.smartCurrentLimit((int) ShooterConstants.LOADING_CURRENT_LIMIT.in(Units.Amps));
        loadingConfig.inverted(ShooterConstants.LOADING_INVERTED);

        loadingMotor.configure(loadingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Shooter Motors (Vortex)
        shooterTopMotor = new SparkFlex(ShooterConstants.SHOOTER_MOTOR_TOP_ID, MotorType.kBrushless);
        shooterTopEncoder = shooterTopMotor.getEncoder();

        shooterBottomMotor = new SparkFlex(ShooterConstants.SHOOTER_MOTOR_BOTTOM_ID, MotorType.kBrushless);
        shooterBottomEncoder = shooterBottomMotor.getEncoder();

        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.idleMode(IdleMode.kBrake);
        shooterConfig.smartCurrentLimit((int) ShooterConstants.SHOOTER_CURRENT_LIMIT.in(Units.Amps));

        // PID Configuration
        shooterConfig.closedLoop.pidf(
                ShooterConstants.SHOOTER_KP,
                ShooterConstants.SHOOTER_KI,
                ShooterConstants.SHOOTER_KD,
                ShooterConstants.SHOOTER_KFF,
                ClosedLoopSlot.kSlot0);

        // Apply Top Config
        shooterConfig.inverted(ShooterConstants.SHOOTER_TOP_INVERTED);
        shooterTopMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Apply Bottom Config
        shooterConfig.inverted(ShooterConstants.SHOOTER_BOTTOM_INVERTED);
        shooterBottomMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Loading
        inputs.loadingPositionRad = Units.Rotations.of(loadingEncoder.getPosition()).in(Units.Radians);
        inputs.loadingVelocityRadPerSec = Units.RadiansPerSecond.of(loadingEncoder.getVelocity())
                .in(Units.RadiansPerSecond);
        inputs.loadingAppliedVolts = loadingMotor.getAppliedOutput() * loadingMotor.getBusVoltage();
        inputs.loadingSupplyCurrentAmps = loadingMotor.getOutputCurrent();
        inputs.loadingTempCelcius = loadingMotor.getMotorTemperature();

        // Shooter Top
        inputs.shooterTopPositionRad = Units.Rotations.of(shooterTopEncoder.getPosition()).in(Units.Radians);
        inputs.shooterTopVelocityRadPerSec = Units.RadiansPerSecond.of(shooterTopEncoder.getVelocity())
                .in(Units.RadiansPerSecond);
        inputs.shooterTopAppliedVolts = shooterTopMotor.getAppliedOutput() * shooterTopMotor.getBusVoltage();
        inputs.shooterTopSupplyCurrentAmps = shooterTopMotor.getOutputCurrent();
        inputs.shooterTopTempCelcius = shooterTopMotor.getMotorTemperature();

        // Shooter Bottom
        inputs.shooterBottomPositionRad = Units.Rotations.of(shooterBottomEncoder.getPosition()).in(Units.Radians);
        inputs.shooterBottomVelocityRadPerSec = Units.RadiansPerSecond.of(shooterBottomEncoder.getVelocity())
                .in(Units.RadiansPerSecond);
        inputs.shooterBottomAppliedVolts = shooterBottomMotor.getAppliedOutput() * shooterBottomMotor.getBusVoltage();
        inputs.shooterBottomSupplyCurrentAmps = shooterBottomMotor.getOutputCurrent();
        inputs.shooterBottomTempCelcius = shooterBottomMotor.getMotorTemperature();
    }

    @Override
    public void setLoadingVoltage(double volts) {
        loadingMotor.setVoltage(volts);
    }

    @Override
    public void setShooterVoltage(double volts) {
        shooterTopMotor.setVoltage(volts);
        shooterBottomMotor.setVoltage(volts);
    }

    @Override
    public void setShooterVelocity(double rpm) {
        // Set velocity for both motors
        // Usually we control one and follower follows, but user asked for "independent
        // controlled, but 2 as a group".
        // Since we set them to same speed, we can just command both.
        shooterTopMotor.getClosedLoopController().setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        shooterBottomMotor.getClosedLoopController().setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void stop() {
        loadingMotor.stopMotor();
        shooterTopMotor.stopMotor();
        shooterBottomMotor.stopMotor();
    }
}
