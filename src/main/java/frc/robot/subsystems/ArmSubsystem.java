// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.math.Gearbox;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem mInstance;

  /* MOTORS & ENCODER */
  private final TalonFX armMotor = new TalonFX(ArmConstants.MOTOR_ID, "CANivore");
  private final DutyCycleEncoder boreEncoder = new DutyCycleEncoder(ArmConstants.BORE_ID);

  public Gearbox armGearbox = ArmConstants.gearRatio;

  /** Checking elapsed time for absolute calibration */
  private final Timer m_timer = new Timer();

  TalonFXConfiguration configs;

  /**
   * Angle to go by the arm
   * unit: degrees
   */
  private double setpoint = 0;

  private final DutyCycleOut m_percentOut = new DutyCycleOut(0);

  /**
   * Output to set in open loop
   * unit: percent
   */
  private double targetOutput = 0;

  private ArmControlState armControlState = ArmControlState.HOLD;

  /** unit: rotations */
  private double lastDemandedRotation;

  /** unit: rotations */
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  private NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

  public enum ArmControlState {
    /** open-loop control */
    OPEN_LOOP,

    /** intaking position */
    INTAKE,

    /** near speaker shooting position - tunable */
    SPEAKER_SHORT,

    /** far from speaker shooting position - NOT tunable */
    SPEAKER_LONG,

    /** amp shooting position */
    AMP,

    /** neutral - in brake */
    HOLD,

    /** zero position with respect to hard stop */
    ZERO,

    /** CLIMBING CLOSED POSITION */
    CLIMB,

    /** intake from source zone when intake is broken */
    INTAKE_FROM_SOURCE,

    /** note pass */
    PASS_NOTE,
  }

  public ArmSubsystem() {

    motorConfig();
    lastDemandedRotation = getArmAngleFalcon();

    /** sets Bore reading to the desired "zero" position */
    boreEncoder.setPositionOffset(ArmConstants.POSITION_OFFSET);

    m_timer.reset();
    m_timer.start();

  }

  public static ArmSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new ArmSubsystem();
    }
    return mInstance;
  }

  private void motorConfig() {
    armMotor.getConfigurator().apply(new TalonFXConfiguration());
    configs = new TalonFXConfiguration();

    configs.Slot0.kP = ArmConstants.kP;
    configs.Slot0.kI = ArmConstants.kI;
    configs.Slot0.kD = ArmConstants.kD;
    configs.Slot0.kS = ArmConstants.kS;
    configs.Slot0.kV = ArmConstants.kV;
    configs.Slot0.kA = ArmConstants.kA;

    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 180;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 180;

    configs.MotionMagic.MotionMagicAcceleration = 200;
    configs.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.ARM_VELOCITY;

    armMotor.getConfigurator().apply(configs);
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public boolean isBoreEncoderAlive() {
    return boreEncoder.isConnected();
    // return false;
  }

  public double getAngle() {
    double angle = 0;

    if (isBoreEncoderAlive()) {
      angle = getArmAngleBore();
    } else {
      angle = getArmAngleFalcon();
    }

    angle = DriverStation.isAutonomous() ? angle + 0.8 : angle;
    // angle=angle-18;
    return angle;
  }

  public boolean isAtSetpoint() {
    if (isBoreEncoderAlive()) {
      return isAtSetpointBore();
    } else {
      return isAtSetpointFalcon();
    }
  }

  @Override
  public void periodic() {
    if (getAngle() > ArmConstants.LAST_RESORT_ANGLE_CUTOFF) {
      armMotor.stopMotor();
      armControlState = ArmControlState.INTAKE;
    }

    boolean shouldStopResetAccordingToBore = isBoreEncoderAlive() && isAtSetpointBore() && !isAtSetpointFalcon();

    if (shouldStopResetAccordingToBore) {
      armMotor.stopMotor();
      resetToAbsolute();
    }

    switch (armControlState) {
      case OPEN_LOOP:
        setMotorOutput();
        break;
      case SPEAKER_LONG:
        setArmAngleMotionMagic(ArmConstants.SPEAKER_LONG);
        break;
      case AMP:
        setArmAngleMotionMagic(ArmConstants.AMP);
        break;
      case INTAKE:
        setArmAngleMotionMagic(ArmConstants.INTAKE);
        break;
      case HOLD:
        armMotor.setControl(new NeutralOut());
        break;
      case ZERO:
        setArmAngleMotionMagic(ArmConstants.ZERO);
        break;
      case INTAKE_FROM_SOURCE:
        setArmAngleMotionMagic(ArmConstants.FROM_INTAKE);
        break;
      case CLIMB:
        setArmAngleMotionMagic(ArmConstants.CLIMB);
        break;
      case PASS_NOTE:
        setArmAngleMotionMagic(ArmConstants.NOTE_PASS);
        break;
      default:
        setArmPercentOutput(0.0);
        break;
    }

    lastDemandedRotation = getArmAngleFalcon();

    SmartDashboard.putBoolean("Is Bore Connected Arm", isBoreEncoderAlive());
    SmartDashboard.putNumber("Arm Falcon", Conversions.revolutionsToDegrees(getArmAngleFalcon()));

    if (shouldStopResetAccordingToBore) {
      return;
    }

  }

  /** toggles neutral mode of motor */
  public void setNeutralMode() {
    this.kNeutralMode = (kNeutralMode == NeutralModeValue.Brake) ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    armMotor.setNeutralMode(this.kNeutralMode);
  }

  /** resets Falcon encoder to zero */
  public void resetFalconEncoder(double desiredAngle) {
    var desPosition = Conversions.degreesToRevolutions(desiredAngle);
    armMotor.setPosition(desPosition);
  }

  // resets falcon encoder to the bore reading so that bore and falcon have the
  // same initial reading
  // theoretically, Falcon position / 119.5 = Bore encoder position + offset at
  // all times
  public void resetToAbsolute() {
    double angle = getArmAngleBore();
    var position = armGearbox.drivenToDriving(angle);
    armMotor.setPosition(position);
  }

  /** @return true if within angle tolerance - FALCON */
  public boolean isAtSetpointFalcon() {
    if (armControlState == ArmControlState.OPEN_LOOP) {
      return false;
    }
    return Math.abs(getArmAngleFalcon() - Conversions.degreesToRevolutions(setpoint)) < ArmConstants.ANGLE_TOLERANCE;
  }

  /** @return true if 0 is within angle tolerance - FALCON */
  public boolean isAtZeroFalcon() {
    return armGearbox.drivingToDriven(armMotor.getRotorPosition().getValue()) < ArmConstants.ANGLE_TOLERANCE;
  }

  /** @return true if within angle tolerance - BORE */
  public boolean isAtSetpointBore() {
    if (armControlState == ArmControlState.OPEN_LOOP) {
      return false;
    }
    return Math.abs(getArmAngleBore()
        - Conversions.degreesToRevolutions(setpoint)) < ArmConstants.ANGLE_TOLERANCE;
  }

  /** @return setpoint unit: degrees */
  public double getSetpoint() {
    return setpoint;
  }

  /** basically "zeroes" arm */
  public double zeroSetpoint() {
    return setpoint = 0;
  }

  /** unit: revolutions */
  public double getArmAngleFalcon() {
    return armGearbox.drivingToDriven(armMotor.getPosition().getValueAsDouble());
  }

  /** unit: revolutions */
  public double getArmAngleBore() {
    return boreEncoder.getAbsolutePosition() - boreEncoder.getPositionOffset();
  }

  public void setArmAngleMotionMagic(double target) {
    setpoint = target;
    armMotor.setControl(motionMagicVoltage.withPosition(
        armGearbox.drivenToDriving(Conversions.degreesToRevolutions(setpoint)))
        .withFeedForward((ArmConstants.kF)
            * Math.cos(Math.toRadians(setpoint))));
  }

  public void setArmVoltage(double voltage) {
    armMotor.setVoltage(voltage);
    lastDemandedRotation = getArmAngleFalcon();
  }

  public void setArmPercentOutput(double percent) {
    armControlState = ArmControlState.OPEN_LOOP;
    targetOutput = percent;
    armMotor.setControl(m_percentOut.withOutput(targetOutput));
    lastDemandedRotation = getArmAngleFalcon();
  }

  public void hold() {
    armControlState = ArmControlState.HOLD;
    armMotor.setControl(new NeutralOut());
  }

  /** unit: rotation */
  public void updateLastDemandedRotation(double rotation) {
    lastDemandedRotation = rotation;
  }

  public void setArmControlState(ArmControlState state) {
    armControlState = state;
  }

  public ArmControlState getArmControlState() {
    return armControlState;
  }

  public void setMotorOutput() {
    armMotor.setControl(m_percentOut.withOutput(targetOutput));
  }

  public TalonFX getArmMotor() {
    return armMotor;
  }

  public double getLastDemandedRotation() {
    return lastDemandedRotation;
  }

}