// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorLevel;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Declara los subsistemas, controles y asignaciones de botones del robot.
 */
public class RobotContainer implements AutoCloseable {
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

  // Si usas un control PS5 (DualSense) nativo en Driver Station, puedes cambiar
  // CommandPS4Controller por CommandPS5Controller manteniendo los mismos ejes y botones frontales.
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  /** Crea el contenedor del robot y configura los enlaces de control. */
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_elevator.setDefaultCommand(m_elevator.stopCommand());

    // Control manual lento con ambos joysticks del control de PlayStation:
    // ambos hacia abajo -> sube lentamente; ambos hacia arriba -> baja lentamente.
    Trigger dualStickManualTrigger =
        new Trigger(
            () ->
                ElevatorSubsystem.isDualStickManualActive(
                    m_driverController.getLeftY(), m_driverController.getRightY()));
    dualStickManualTrigger.whileTrue(
        m_elevator.manualDualStickCommand(
            m_driverController::getLeftY, m_driverController::getRightY));

    // Posiciones predefinidas del elevador (todas <= 0.85 m) en los botones frontales de PS.
    m_driverController.cross().onTrue(m_elevator.goToLevelCommand(ElevatorLevel.INICIAL));
    m_driverController.square().onTrue(m_elevator.goToLevelCommand(ElevatorLevel.L1));
    m_driverController.circle().onTrue(m_elevator.goToLevelCommand(ElevatorLevel.L2));
    m_driverController.triangle().onTrue(m_elevator.goToLevelCommand(ElevatorLevel.L3));

    // Reiniciar encoders a 0.0 para pruebas en banco.
    m_driverController.options().onTrue(m_elevator.zeroEncodersCommand());
  }

  /**
   * Comando autonomo (sin rutina autonoma por defecto en este prototipo).
   *
   * @return Comando vacio.
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }

  /** Devuelve el subsistema del elevador (util para pruebas). */
  public ElevatorSubsystem getElevatorSubsystem() {
    return m_elevator;
  }

  @Override
  public void close() {
    m_elevator.close();
  }
}
