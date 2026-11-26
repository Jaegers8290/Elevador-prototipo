// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

/**
 * Constantes globales del robot y del prototipo del elevador.
 */
public final class Constants {
  private Constants() {}

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kJoystickDeadband = 0.15;

    private OperatorConstants() {}
  }

  public static final class ElevatorConstants {
    public static final int kLeaderMotorId = 7;
    public static final int kFollowerMotorId = 8;

    // Direccion base del motor 7 y alineacion del motor 8 respecto al lider.
    // Cambiar kFollowerAlignment a MotorAlignmentValue.Opposed si al montar la mecanica giran
    // opuestos.
    public static final InvertedValue kLeaderInverted = InvertedValue.CounterClockwise_Positive;
    public static final MotorAlignmentValue kFollowerAlignment = MotorAlignmentValue.Aligned;

    // Desactivado por defecto para funcionar sin licencia Phoenix Pro.
    // Cambiar a true si ambos Kraken X60 tienen licencia Phoenix Pro activa.
    public static final boolean kEnableFOC = false;

    // Transmision 25:1 en cada motor individual (25 vueltas del motor por 1 vuelta de la flecha).
    public static final double kGearRatio = 25.0;

    // Configuracion continua (no cascada): el carro avanza 1:1 con el recorrido de la banda.
    // Longitud de referencia de la banda por lado/motor: 3.25 m.
    public static final double kBeltLengthPerSideMeters = 3.25;

    // Avance lineal por vuelta de la flecha de salida (circunferencia de la polea motriz, sin
    // multiplicador de cascada porque es continuo).
    // Actualizar con (dientes_polea * paso_banda) o (PI * diametro_polea), p. ej. 20T HTD 5mm = 0.10 m.
    public static final double kMetersPerMechanismRotation = 0.10;

    // Masa aproximada del carro + carga (~30 kg, valor preliminar sujeto a cambios).
    public static final double kCarriageMassKg = 30.0;
    // Radio equivalente de polea/sprocket consistente con kMetersPerMechanismRotation.
    public static final double kDrumRadiusMeters = kMetersPerMechanismRotation / (2.0 * Math.PI);

    // Limites por software desactivados mientras se prueban motores sueltos en banco.
    // Activar en true cuando el elevador este ensamblado y puesto en cero abajo.
    public static final boolean kEnableSoftLimits = false;
    public static final double kMinHeightMeters = 0.0;
    public static final double kMaxHeightMeters = 0.95;

    // Limites de corriente para Kraken X60 considerando carga aproximada de 30 kg.
    public static final double kSupplyCurrentLimitAmps = 40.0;
    public static final double kStatorCurrentLimitAmps = 80.0;

    // Ganancias iniciales de Slot 0 ajustadas para carga estimada de 30 kg y transmision 25:1.
    // Ajustar kG, kP y kV en banco/robot una vez definida la polea y el peso final.
    public static final double kP = 6.0;
    public static final double kI = 0.0;
    public static final double kD = 0.10;
    public static final double kS = 0.15;
    public static final double kV = 3.0;
    public static final double kG = 0.22;

    // Perfil Motion Magic lento y seguro para pruebas iniciales.
    public static final double kMotionMagicCruiseVelocityRps = 2.0;
    public static final double kMotionMagicAccelerationRpsPerSec = 4.0;

    // Limite de velocidad manual con ambos joysticks (0.0 a 1.0, p. ej. 0.50 = 50% maximo).
    // Cambia este valor si quieres que suba/baje mas lento o mas rapido en modo manual.
    public static final double kManualSlowMaxDutyCycle = 0.50;

    public static final double kPositionToleranceMeters = 0.01;
    public static final String kShuffleboardTabName = "Elevador";

    private ElevatorConstants() {}
  }

  /**
   * Posiciones predefinidas de prueba para el elevador (todas por debajo de 1.0 metro).
   */
  public enum ElevatorLevel {
    INICIAL("Inicial", 0.00),
    L1("L1", 0.25),
    L2("L2", 0.55),
    L3("L3", 0.85);

    private final String m_displayName;
    private final double m_heightMeters;

    ElevatorLevel(String displayName, double heightMeters) {
      m_displayName = displayName;
      m_heightMeters = heightMeters;
    }

    public String getDisplayName() {
      return m_displayName;
    }

    public double getHeightMeters() {
      return m_heightMeters;
    }
  }
}
