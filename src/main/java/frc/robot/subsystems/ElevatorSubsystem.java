// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorLevel;
import frc.robot.Constants.OperatorConstants;
import java.util.Locale;
import java.util.function.DoubleSupplier;

/**
 * Subsistema prototipo del elevador con dos motores Kraken X60 (IDs 7 y 8).
 */
public class ElevatorSubsystem extends SubsystemBase implements AutoCloseable {
  private static final double kSimLoopPeriodSeconds = 0.020;

  private final TalonFX m_leaderMotor;
  private final TalonFX m_followerMotor;

  private final StatusSignal<Angle> m_leaderPositionSignal;
  private final StatusSignal<Angle> m_followerPositionSignal;
  private final StatusSignal<AngularVelocity> m_leaderVelocitySignal;
  private final StatusSignal<AngularVelocity> m_followerVelocitySignal;

  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0.0)
      .withEnableFOC(ElevatorConstants.kEnableFOC);
  private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0.0).withEnableFOC(ElevatorConstants.kEnableFOC);
  private final NeutralOut m_neutralRequest = new NeutralOut();
  private final Follower m_followerRequest = new Follower(ElevatorConstants.kLeaderMotorId,
      ElevatorConstants.kFollowerAlignment);

  private final GenericEntry m_motor7MetersEntry;
  private final GenericEntry m_motor8MetersEntry;
  private final GenericEntry m_motor7RotationsEntry;
  private final GenericEntry m_motor8RotationsEntry;
  private final GenericEntry m_targetMetersEntry;
  private final GenericEntry m_statusEntry;
  private final GenericEntry m_manualActiveEntry;

  private final DCMotorSim m_leaderMotorSim;
  private final DCMotorSim m_followerMotorSim;

  private double m_targetMeters = ElevatorLevel.INICIAL.getHeightMeters();
  private String m_statusLabel = "Detenido";
  private boolean m_manualControlActive = false;

  /** Crea el subsistema usando la pestana de Shuffleboard por defecto. */
  public ElevatorSubsystem() {
    this(ElevatorConstants.kShuffleboardTabName);
  }

  /**
   * Crea el subsistema especificando el nombre de la pestana de Shuffleboard.
   *
   * @param shuffleboardTabName Nombre de la pestana en Shuffleboard.
   */
  public ElevatorSubsystem(String shuffleboardTabName) {
    m_leaderMotor = new TalonFX(ElevatorConstants.kLeaderMotorId);
    m_followerMotor = new TalonFX(ElevatorConstants.kFollowerMotorId);

    DCMotor dualKrakenModel = DCMotor.getKrakenX60(2);
    double equivalentLoadMoiKgM2 = ElevatorConstants.kCarriageMassKg
        * ElevatorConstants.kDrumRadiusMeters
        * ElevatorConstants.kDrumRadiusMeters;
    m_leaderMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            dualKrakenModel, equivalentLoadMoiKgM2, ElevatorConstants.kGearRatio),
        dualKrakenModel);
    m_followerMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            dualKrakenModel, equivalentLoadMoiKgM2, ElevatorConstants.kGearRatio),
        dualKrakenModel);

    if (RobotBase.isSimulation()) {
      m_leaderMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
      m_followerMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    configureMotors();
    zeroEncoders();

    m_leaderPositionSignal = m_leaderMotor.getPosition();
    m_followerPositionSignal = m_followerMotor.getPosition();
    m_leaderVelocitySignal = m_leaderMotor.getVelocity();
    m_followerVelocitySignal = m_followerMotor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_leaderPositionSignal,
        m_followerPositionSignal,
        m_leaderVelocitySignal,
        m_followerVelocitySignal);

    m_followerMotor.setControl(m_followerRequest);

    ShuffleboardTab tab = Shuffleboard.getTab(shuffleboardTabName);
    m_motor7MetersEntry = getOrCreateEntry(tab, "Motor 7 Posicion (m)", 0.0, 0, 0);
    m_motor8MetersEntry = getOrCreateEntry(tab, "Motor 8 Posicion (m)", 0.0, 2, 0);
    m_motor7RotationsEntry = getOrCreateEntry(tab, "Motor 7 Rotaciones", 0.0, 0, 1);
    m_motor8RotationsEntry = getOrCreateEntry(tab, "Motor 8 Rotaciones", 0.0, 2, 1);
    m_targetMetersEntry = getOrCreateEntry(tab, "Posicion Objetivo (m)", 0.0, 0, 2);
    m_statusEntry = getOrCreateEntry(tab, "Modo / Nivel", m_statusLabel, 2, 2);
    m_manualActiveEntry = getOrCreateEntry(tab, "Control Manual Activo", false, 4, 0);

    addCommandIfAbsent(
        tab, "Ir a Inicial (0.0 m)", goToLevelCommand(ElevatorLevel.INICIAL), 4, 1);
    addCommandIfAbsent(tab, "Ir a L1 (0.25 m)", goToLevelCommand(ElevatorLevel.L1), 4, 2);
    addCommandIfAbsent(tab, "Ir a L2 (0.55 m)", goToLevelCommand(ElevatorLevel.L2), 4, 3);
    addCommandIfAbsent(tab, "Ir a L3 (0.85 m)", goToLevelCommand(ElevatorLevel.L3), 4, 4);
    addCommandIfAbsent(tab, "Cero Encoders", zeroEncodersCommand(), 0, 3);
  }

  private void configureMotors() {
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.MotorOutput.Inverted = ElevatorConstants.kLeaderInverted;
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    leaderConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.kGearRatio;

    leaderConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.kSupplyCurrentLimitAmps;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kStatorCurrentLimitAmps;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    leaderConfig.Slot0.kP = ElevatorConstants.kP;
    leaderConfig.Slot0.kI = ElevatorConstants.kI;
    leaderConfig.Slot0.kD = ElevatorConstants.kD;
    leaderConfig.Slot0.kS = ElevatorConstants.kS;
    leaderConfig.Slot0.kV = ElevatorConstants.kV;
    leaderConfig.Slot0.kG = ElevatorConstants.kG;
    leaderConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    leaderConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kMotionMagicCruiseVelocityRps;
    leaderConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kMotionMagicAccelerationRpsPerSec;

    leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = ElevatorConstants.kEnableSoftLimits;
    leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = metersToMechanismRotations(
        ElevatorConstants.kMinHeightMeters);
    leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = ElevatorConstants.kEnableSoftLimits;
    leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = metersToMechanismRotations(
        ElevatorConstants.kMaxHeightMeters);

    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followerConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.kGearRatio;
    followerConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.kSupplyCurrentLimitAmps;
    followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    followerConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kStatorCurrentLimitAmps;
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    applyConfigWithWarning(m_leaderMotor, leaderConfig, ElevatorConstants.kLeaderMotorId);
    applyConfigWithWarning(m_followerMotor, followerConfig, ElevatorConstants.kFollowerMotorId);
  }

  private static void applyConfigWithWarning(
      TalonFX motor, TalonFXConfiguration config, int motorId) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int attempt = 0; attempt < 5; attempt++) {
      status = motor.getConfigurator().apply(config, 0.150);
      if (status.isOK()) {
        return;
      }
      if (RobotBase.isSimulation()) {
        try {
          Thread.sleep(25);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          break;
        }
      }
    }
    DriverStation.reportWarning(
        "No se pudo configurar Kraken X60 ID " + motorId + ": " + status.getDescription(), false);
  }

  private static GenericEntry getOrCreateEntry(
      ShuffleboardTab tab, String title, Object defaultValue, int column, int row) {
    for (ShuffleboardComponent<?> component : tab.getComponents()) {
      if (component instanceof SimpleWidget widget && widget.getTitle().equals(title)) {
        return widget.getEntry();
      }
    }
    return tab.add(title, defaultValue).withPosition(column, row).withSize(2, 1).getEntry();
  }

  private static void addCommandIfAbsent(
      ShuffleboardTab tab, String title, Command command, int column, int row) {
    boolean exists = tab.getComponents().stream().anyMatch(component -> component.getTitle().equals(title));
    if (!exists) {
      tab.add(title, command).withPosition(column, row).withSize(2, 1);
    }
  }

  /** Convierte metros de recorrido a rotaciones del mecanismo de salida. */
  public static double metersToMechanismRotations(double meters) {
    return meters / ElevatorConstants.kMetersPerMechanismRotation;
  }

  /** Convierte rotaciones del mecanismo de salida a metros. */
  public static double mechanismRotationsToMeters(double rotations) {
    return rotations * ElevatorConstants.kMetersPerMechanismRotation;
  }

  /**
   * Indica si ambos joysticks estan presionados en la misma direccion (ambos
   * abajo o ambos
   * arriba) fuera de la zona muerta.
   */
  public static boolean isDualStickManualActive(double leftY, double rightY) {
    double deadband = OperatorConstants.kJoystickDeadband;
    boolean bothDown = leftY > deadband && rightY > deadband;
    boolean bothUp = leftY < -deadband && rightY < -deadband;
    return bothDown || bothUp;
  }

  /**
   * Calcula el duty cycle lento para el elevador a partir de los dos joysticks de
   * PlayStation.
   *
   * <p>
   * En WPILib (PS4/PS5), empujar el joystick hacia abajo da valores positivos y
   * empujarlo hacia
   * arriba da valores negativos:
   * <ul>
   * <li>Ambos hacia abajo ({@code leftY > deadband && rightY > deadband}): sube
   * lentamente ({@code > 0}).
   * <li>Ambos hacia arriba ({@code leftY < -deadband && rightY < -deadband}):
   * baja lentamente ({@code < 0}).
   * <li>Si solo se mueve uno o estan en direcciones opuestas: devuelve
   * {@code 0.0}.
   * </ul>
   */
  public static double computeDualStickSlowOutput(double leftY, double rightY) {
    double deadband = OperatorConstants.kJoystickDeadband;
    double maxDutyCycle = MathUtil.clamp(ElevatorConstants.kManualSlowMaxDutyCycle, 0.0, 1.0);
    if (leftY > deadband && rightY > deadband) {
      double combined = Math.min(leftY, rightY);
      return MathUtil.applyDeadband(combined, deadband) * maxDutyCycle;
    }
    if (leftY < -deadband && rightY < -deadband) {
      double combined = Math.max(leftY, rightY);
      return MathUtil.applyDeadband(combined, deadband) * maxDutyCycle;
    }
    return 0.0;
  }

  /** Envia al elevador a una posicion objetivo en metros usando Motion Magic. */
  public void setTargetMeters(double meters) {
    m_targetMeters = meters;
    m_leaderMotor.setControl(
        m_motionMagicRequest.withPosition(metersToMechanismRotations(meters)));
    m_followerMotor.setControl(m_followerRequest);
  }

  /** Aplica salida manual en lazo abierto (duty cycle) al lider y al seguidor. */
  public void setManualDutyCycle(double dutyCycle) {
    m_leaderMotor.setControl(m_dutyCycleRequest.withOutput(dutyCycle));
    m_followerMotor.setControl(m_followerRequest);
  }

  /** Detiene ambos motores en modo neutral (Brake). */
  public void stop() {
    m_manualControlActive = false;
    m_statusLabel = "Detenido";
    m_leaderMotor.setControl(m_neutralRequest);
    m_followerMotor.setControl(m_followerRequest);
  }

  /** Reinicia a 0.0 la posicion reportada de ambos encoders (ID 7 e ID 8). */
  public final void zeroEncoders() {
    m_leaderMotor.setPosition(0.0);
    m_followerMotor.setPosition(0.0);
    m_targetMeters = 0.0;
    if (RobotBase.isSimulation()) {
      m_leaderMotorSim.setState(0.0, 0.0);
      m_followerMotorSim.setState(0.0, 0.0);
      m_leaderMotor.getSimState().setRawRotorPosition(0.0);
      m_leaderMotor.getSimState().setRotorVelocity(0.0);
      m_followerMotor.getSimState().setRawRotorPosition(0.0);
      m_followerMotor.getSimState().setRotorVelocity(0.0);
    }
  }

  /** Posicion actual del motor 7 (lider) en rotaciones de mecanismo. */
  public double getLeaderRotations() {
    return m_leaderPositionSignal.getValueAsDouble();
  }

  /** Posicion actual del motor 8 (seguidor) en rotaciones de mecanismo. */
  public double getFollowerRotations() {
    return m_followerPositionSignal.getValueAsDouble();
  }

  /** Posicion actual del motor 7 (lider) en metros. */
  public double getLeaderPositionMeters() {
    return mechanismRotationsToMeters(getLeaderRotations());
  }

  /** Posicion actual del motor 8 (seguidor) en metros. */
  public double getFollowerPositionMeters() {
    return mechanismRotationsToMeters(getFollowerRotations());
  }

  /** Ultima altura objetivo solicitada en metros. */
  public double getTargetMeters() {
    return m_targetMeters;
  }

  /**
   * Devuelve true si el motor lider esta dentro de la tolerancia del objetivo.
   */
  public boolean isAtTarget() {
    return Math.abs(getLeaderPositionMeters() - m_targetMeters) <= ElevatorConstants.kPositionToleranceMeters;
  }

  /**
   * Comando continuo que lleva y mantiene el elevador en un nivel predefinido.
   */
  public Command goToLevelCommand(ElevatorLevel level) {
    return run(
        () -> {
          m_manualControlActive = false;
          m_statusLabel = String.format(
              Locale.US, "%s (%.2f m)", level.getDisplayName(), level.getHeightMeters());
          setTargetMeters(level.getHeightMeters());
        })
        .withName("Ir a " + level.getDisplayName());
  }

  /**
   * Comando de control manual lento usando ambos joysticks del control de
   * PlayStation.
   */
  public Command manualDualStickCommand(
      DoubleSupplier leftYSupplier, DoubleSupplier rightYSupplier) {
    return run(
        () -> {
          double output = computeDualStickSlowOutput(
              leftYSupplier.getAsDouble(), rightYSupplier.getAsDouble());
          m_manualControlActive = Math.abs(output) > 1e-6;
          if (output > 0.0) {
            m_statusLabel = "Manual Subiendo";
          } else if (output < 0.0) {
            m_statusLabel = "Manual Bajando";
          } else {
            m_statusLabel = "Manual Detenido";
          }
          m_targetMeters = getLeaderPositionMeters();
          setManualDutyCycle(output);
        })
        .finallyDo(this::stop)
        .withName("Manual Doble Joystick");
  }

  /** Comando por defecto que mantiene los motores detenidos en modo Brake. */
  public Command stopCommand() {
    return run(this::stop).withName("Elevador Detenido");
  }

  /** Comando que reinicia ambos encoders a 0.0 incluso estando deshabilitado. */
  public Command zeroEncodersCommand() {
    return runOnce(this::zeroEncoders).ignoringDisable(true).withName("Cero Encoders");
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        m_leaderPositionSignal,
        m_followerPositionSignal,
        m_leaderVelocitySignal,
        m_followerVelocitySignal);

    m_motor7MetersEntry.setDouble(getLeaderPositionMeters());
    m_motor8MetersEntry.setDouble(getFollowerPositionMeters());
    m_motor7RotationsEntry.setDouble(getLeaderRotations());
    m_motor8RotationsEntry.setDouble(getFollowerRotations());
    m_targetMetersEntry.setDouble(m_targetMeters);
    m_statusEntry.setString(m_statusLabel);
    m_manualActiveEntry.setBoolean(m_manualControlActive);
  }

  @Override
  public void simulationPeriodic() {
    double batteryVoltage = RobotController.getBatteryVoltage();

    updateMotorSim(m_leaderMotor.getSimState(), m_leaderMotorSim, batteryVoltage);
    updateMotorSim(m_followerMotor.getSimState(), m_followerMotorSim, batteryVoltage);
  }

  private static void updateMotorSim(
      TalonFXSimState simState, DCMotorSim motorSim, double batteryVoltage) {
    simState.setSupplyVoltage(batteryVoltage);
    motorSim.setInputVoltage(simState.getMotorVoltage());
    motorSim.update(kSimLoopPeriodSeconds);
    simState.setRawRotorPosition(
        motorSim.getAngularPositionRotations() * ElevatorConstants.kGearRatio);
    simState.setRotorVelocity(
        (motorSim.getAngularVelocityRPM() / 60.0) * ElevatorConstants.kGearRatio);
  }

  @Override
  public void close() {
    CommandScheduler.getInstance().unregisterSubsystem(this);
    m_leaderMotor.close();
    m_followerMotor.close();
  }
}
