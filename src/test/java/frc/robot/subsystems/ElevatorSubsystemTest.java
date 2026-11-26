// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.PS4ControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorLevel;
import frc.robot.Constants.OperatorConstants;
import frc.robot.RobotContainer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ElevatorSubsystemTest {
  private static final int kSimStepMillis = 20;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().getDefaultButtonLoop().clear();
    CommandScheduler.getInstance().clearComposedCommands();

    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setJoystickAxisCount(OperatorConstants.kDriverControllerPort, 6);
    DriverStationSim.setJoystickButtonCount(OperatorConstants.kDriverControllerPort, 14);
    for (int axis = 0; axis < 6; axis++) {
      DriverStationSim.setJoystickAxis(OperatorConstants.kDriverControllerPort, axis, 0.0);
    }
    for (int button = 1; button <= 14; button++) {
      DriverStationSim.setJoystickButton(OperatorConstants.kDriverControllerPort, button, false);
    }
    DriverStationSim.notifyNewData();
    Unmanaged.feedEnable(100);
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().getDefaultButtonLoop().clear();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  private static void stepLoop(ElevatorSubsystem elevator, int loops)
      throws InterruptedException {
    for (int i = 0; i < loops; i++) {
      DriverStationSim.notifyNewData();
      Unmanaged.feedEnable(100);
      HAL.simPeriodicBefore();
      CommandScheduler.getInstance().run();
      elevator.simulationPeriodic();
      HAL.simPeriodicAfter();
      Shuffleboard.update();
      Thread.sleep(kSimStepMillis);
    }
  }

  @Test
  void presetLevelsAreUnderOneMeterAndOrdered() {
    assertEquals(0.0, ElevatorLevel.INICIAL.getHeightMeters(), 1e-9);
    assertTrue(ElevatorLevel.L1.getHeightMeters() > ElevatorLevel.INICIAL.getHeightMeters());
    assertTrue(ElevatorLevel.L2.getHeightMeters() > ElevatorLevel.L1.getHeightMeters());
    assertTrue(ElevatorLevel.L3.getHeightMeters() > ElevatorLevel.L2.getHeightMeters());
    assertTrue(ElevatorLevel.L3.getHeightMeters() < 1.0);
  }

  @Test
  void unitConversionsAreConsistent() {
    for (ElevatorLevel level : ElevatorLevel.values()) {
      double rotations = ElevatorSubsystem.metersToMechanismRotations(level.getHeightMeters());
      double roundTripMeters = ElevatorSubsystem.mechanismRotationsToMeters(rotations);
      assertEquals(level.getHeightMeters(), roundTripMeters, 1e-9);
    }
  }

  @Test
  void dualStickSlowControlFollowsPlayStationConvention() {
    // Ambos joysticks hacia abajo (valores positivos en PS4/PS5) -> sube lentamente (> 0).
    double upOutput = ElevatorSubsystem.computeDualStickSlowOutput(0.9, 0.7);
    assertTrue(upOutput > 0.0);
    assertTrue(upOutput <= ElevatorConstants.kManualSlowMaxDutyCycle);
    assertTrue(ElevatorSubsystem.isDualStickManualActive(0.9, 0.7));

    // Ambos joysticks hacia arriba (valores negativos en PS4/PS5) -> baja lentamente (< 0).
    double downOutput = ElevatorSubsystem.computeDualStickSlowOutput(-0.9, -0.7);
    assertTrue(downOutput < 0.0);
    assertTrue(downOutput >= -ElevatorConstants.kManualSlowMaxDutyCycle);
    assertTrue(ElevatorSubsystem.isDualStickManualActive(-0.9, -0.7));

    // Un solo joystick, direcciones opuestas o dentro de deadband -> 0.0.
    assertEquals(0.0, ElevatorSubsystem.computeDualStickSlowOutput(0.8, 0.0), 1e-9);
    assertEquals(0.0, ElevatorSubsystem.computeDualStickSlowOutput(0.0, -0.8), 1e-9);
    assertEquals(0.0, ElevatorSubsystem.computeDualStickSlowOutput(0.8, -0.8), 1e-9);
    assertEquals(0.0, ElevatorSubsystem.computeDualStickSlowOutput(0.1, 0.1), 1e-9);
    assertFalse(ElevatorSubsystem.isDualStickManualActive(0.8, -0.8));
  }

  @Test
  void simulationMovesBothMotorsToLevelsAndPublishesShuffleboard() throws InterruptedException {
    try (ElevatorSubsystem elevator = new ElevatorSubsystem("ElevadorTestNiveles")) {
      NetworkTable table =
          NetworkTableInstance.getDefault().getTable("Shuffleboard/ElevadorTestNiveles");

      Command goToL1 = elevator.goToLevelCommand(ElevatorLevel.L1);
      CommandScheduler.getInstance().schedule(goToL1);
      stepLoop(elevator, 120);

      assertEquals(ElevatorLevel.L1.getHeightMeters(), elevator.getLeaderPositionMeters(), 0.03);
      assertEquals(
          elevator.getLeaderPositionMeters(), elevator.getFollowerPositionMeters(), 0.05);
      assertEquals(
          elevator.getLeaderPositionMeters(),
          table.getEntry("Motor 7 Posicion (m)").getDouble(-1.0),
          1e-4);
      assertEquals(
          elevator.getFollowerPositionMeters(),
          table.getEntry("Motor 8 Posicion (m)").getDouble(-1.0),
          1e-4);
      assertEquals(
          ElevatorLevel.L1.getHeightMeters(),
          table.getEntry("Posicion Objetivo (m)").getDouble(-1.0),
          1e-4);

      Command goToInicial = elevator.goToLevelCommand(ElevatorLevel.INICIAL);
      CommandScheduler.getInstance().schedule(goToInicial);
      stepLoop(elevator, 120);

      assertEquals(
          ElevatorLevel.INICIAL.getHeightMeters(), elevator.getLeaderPositionMeters(), 0.03);
      assertEquals(
          ElevatorLevel.INICIAL.getHeightMeters(), elevator.getFollowerPositionMeters(), 0.05);
    }
  }

  @Test
  void simulationDualStickRaisesWhenDownAndLowersWhenUp() throws InterruptedException {
    try (ElevatorSubsystem elevator = new ElevatorSubsystem("ElevadorTestManual")) {
      CommandPS4Controller controller =
          new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
      PS4ControllerSim ps4Sim = new PS4ControllerSim(OperatorConstants.kDriverControllerPort);
      NetworkTable table =
          NetworkTableInstance.getDefault().getTable("Shuffleboard/ElevadorTestManual");

      // Ambos joysticks hacia abajo (+0.9) -> sube lentamente.
      ps4Sim.setLeftY(0.9);
      ps4Sim.setRightY(0.9);
      Command manualCmd =
          elevator.manualDualStickCommand(controller::getLeftY, controller::getRightY);
      CommandScheduler.getInstance().schedule(manualCmd);
      stepLoop(elevator, 40);

      double heightAfterGoingUp = elevator.getLeaderPositionMeters();
      assertTrue(heightAfterGoingUp > 0.05, "Debe subir cuando ambos sticks estan abajo");
      assertTrue(
          elevator.getFollowerPositionMeters() > 0.05,
          "El motor 8 debe seguir al motor 7 al subir");
      assertEquals("Manual Subiendo", table.getEntry("Modo / Nivel").getString(""));
      assertTrue(table.getEntry("Control Manual Activo").getBoolean(false));

      // Ambos joysticks hacia arriba (-0.9) -> baja lentamente.
      ps4Sim.setLeftY(-0.9);
      ps4Sim.setRightY(-0.9);
      stepLoop(elevator, 60);

      double heightAfterGoingDown = elevator.getLeaderPositionMeters();
      assertTrue(
          heightAfterGoingDown < heightAfterGoingUp,
          "Debe bajar cuando ambos sticks miran hacia arriba");
      assertEquals("Manual Bajando", table.getEntry("Modo / Nivel").getString(""));
    }
  }

  @Test
  void robotContainerBindingsSwitchLevelsAndOverrideWithDualStickManual()
      throws InterruptedException {
    try (RobotContainer container = new RobotContainer()) {
      ElevatorSubsystem elevator = container.getElevatorSubsystem();
      PS4ControllerSim ps4Sim = new PS4ControllerSim(OperatorConstants.kDriverControllerPort);
      NetworkTable table =
          NetworkTableInstance.getDefault().getTable("Shuffleboard/Elevador");

      // Presionar Circulo -> L2 (0.55 m)
      ps4Sim.setCircleButton(true);
      stepLoop(elevator, 2);
      ps4Sim.setCircleButton(false);
      stepLoop(elevator, 200);

      assertEquals(ElevatorLevel.L2.getHeightMeters(), elevator.getLeaderPositionMeters(), 0.03);
      assertEquals(
          elevator.getLeaderPositionMeters(),
          table.getEntry("Motor 7 Posicion (m)").getDouble(-1.0),
          1e-4);

      // Empujar ambos joysticks hacia arriba (-0.9) -> baja lentamente e interrumpe el nivel
      ps4Sim.setLeftY(-0.9);
      ps4Sim.setRightY(-0.9);
      stepLoop(elevator, 40);

      double loweredHeight = elevator.getLeaderPositionMeters();
      assertTrue(loweredHeight < ElevatorLevel.L2.getHeightMeters() - 0.05);
      assertEquals("Manual Bajando", table.getEntry("Modo / Nivel").getString(""));

      // Soltar un joystick -> se detiene el control manual
      ps4Sim.setLeftY(0.0);
      stepLoop(elevator, 10);
      assertEquals("Detenido", table.getEntry("Modo / Nivel").getString(""));
      assertFalse(table.getEntry("Control Manual Activo").getBoolean(true));
    }
  }
}
