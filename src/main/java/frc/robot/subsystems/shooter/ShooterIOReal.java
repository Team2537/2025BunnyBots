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
    private final SparkFlex shooterLeftMotor;
    private final SparkFlex shooterRightMotor;

    private final RelativeEncoder loadingEncoder;
    private final RelativeEncoder shooterLeftEncoder;
    private final RelativeEncoder shooterRightEncoder;

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
        shooterLeftMotor = new SparkFlex(ShooterConstants.SHOOTER_MOTOR_LEFT_ID, MotorType.kBrushless);
        shooterLeftEncoder = shooterLeftMotor.getEncoder();

        shooterRightMotor = new SparkFlex(ShooterConstants.SHOOTER_MOTOR_RIGHT_ID, MotorType.kBrushless);
        shooterRightEncoder = shooterRightMotor.getEncoder();

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

        // Apply Left Config
        shooterConfig.inverted(ShooterConstants.SHOOTER_LEFT_INVERTED);
        shooterLeftMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Apply Right Config
        shooterConfig.inverted(ShooterConstants.SHOOTER_RIGHT_INVERTED);
        shooterRightMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

        // Shooter Left
        inputs.shooterLeftPositionRad = Units.Rotations.of(shooterLeftEncoder.getPosition()).in(Units.Radians);
        inputs.shooterLeftVelocityRadPerSec = Units.RadiansPerSecond.of(shooterLeftEncoder.getVelocity())
                .in(Units.RadiansPerSecond);
        inputs.shooterLeftAppliedVolts = shooterLeftMotor.getAppliedOutput() * shooterLeftMotor.getBusVoltage();
        inputs.shooterLeftSupplyCurrentAmps = shooterLeftMotor.getOutputCurrent();
        inputs.shooterLeftTempCelcius = shooterLeftMotor.getMotorTemperature();

        // Shooter Right
        inputs.shooterRightPositionRad = Units.Rotations.of(shooterRightEncoder.getPosition()).in(Units.Radians);
        inputs.shooterRightVelocityRadPerSec = Units.RadiansPerSecond.of(shooterRightEncoder.getVelocity())
                .in(Units.RadiansPerSecond);
        inputs.shooterRightAppliedVolts = shooterRightMotor.getAppliedOutput() * shooterRightMotor.getBusVoltage();
        inputs.shooterRightSupplyCurrentAmps = shooterRightMotor.getOutputCurrent();
        inputs.shooterRightTempCelcius = shooterRightMotor.getMotorTemperature();
    }

    @Override
    public void setLoadingVoltage(double volts) {
        loadingMotor.setVoltage(volts);
    }

    @Override
    public void setShooterVoltage(double volts) {
        shooterLeftMotor.setVoltage(volts);
        shooterRightMotor.setVoltage(volts);
    }

    @Override
    public void setShooterVelocity(double rpm) {
        // Set velocity for both motors
        // Usually we control one and follower follows, but user asked for "independent
        // controlled, but 2 as a group".
        // Since we set them to same speed, we can just command both.
        shooterLeftMotor.getClosedLoopController().setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        shooterRightMotor.getClosedLoopController().setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void stop() {
        loadingMotor.stopMotor();
        shooterLeftMotor.stopMotor();
        shooterRightMotor.stopMotor();
    }
}
