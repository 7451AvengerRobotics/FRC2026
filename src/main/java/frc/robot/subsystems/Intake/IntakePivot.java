package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;

public class IntakePivot extends SubsystemBase {
    private final TalonFX intakePivot = new TalonFX(IntakePivotConstants.kIntakePivotID);
    private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

    public IntakePivot() {
        super();

        setName("IntakePivot");

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 40;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = IntakePivotConstants.kIntakeGearRatio;
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.75))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(7))
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = IntakePivotConstants.kS;
        slot0.kV = IntakePivotConstants.kV;
        slot0.kA = IntakePivotConstants.kA;
        slot0.kP = IntakePivotConstants.kP;
        slot0.kI = IntakePivotConstants.kI;
        slot0.kD = IntakePivotConstants.kD;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = intakePivot.getConfigurator().apply(cfg);
            if (status.isOK())
            break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        intakePivot.getConfigurator().setPosition(0);

        BaseStatusSignal.setUpdateFrequencyForAll(250,
            intakePivot.getPosition(),
            intakePivot.getVelocity(),
            intakePivot.getMotorVoltage());
    }

    public void pivotIntake(double rotations) {
        intakePivot.setControl(pivotRequest.withPosition(rotations).withSlot(0));
    }

    public boolean endCommand() {
        if (intakePivot.getVelocity(true).getValueAsDouble() == 0.0 && intakePivot.getPosition().getValueAsDouble() > 0.01) {
            return true;
        }
        return false;
    }

    public Command setIntakePivotAngle(PivotPosition pos) {
        return setIntakePivotAngle(pos.rotations).until(() -> nearSetpoint(pos));
    }

    public Command setIntakePivotAngle(double rotations) {
        return run(() -> {
            pivotIntake(rotations);
        });
    }

    public Command toPosition(Supplier<PivotPosition> pos) {
        return setIntakePivotAngle(pos.get());
    }

    public boolean nearSetpoint(PivotPosition pos) {
        double diff = intakePivot.getPosition().getValueAsDouble() - pos.rotations;
        return Math.abs(diff) <= 0.05;
    }

    public boolean atStow() {
        return nearSetpoint(PivotPosition.STOW);
    }

    public boolean atDeployed() {
        return nearSetpoint(PivotPosition.DEPLOYED);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakePivot Rotations", intakePivot.getPosition().getValueAsDouble());
    }

    public enum PivotPosition {
        STOW(0.05),
        DEPLOYED(0.35);

        public final double rotations;

        private PivotPosition(double rotations) {
            this.rotations = rotations;
        }
    }
}
