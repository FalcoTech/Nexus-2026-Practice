// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CAN_IDs;
// import frc.robot.Constants.CurrentLimits;
import java.util.Optional;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Dual-motor flywheel shooter subsystem. The left NEO drives via closed-loop velocity control (YAMS
 * {@link yams.mechanisms.velocity.FlyWheel}), and the right NEO follows inverted.
 *
 * <p>Velocity setpoints are typically provided by {@link ShotCalculator} during competition, or set
 * manually via {@code manualRPM} in {@link frc.robot.RobotContainer} for tuning.
 */
public class Shooter extends SubsystemBase {

  private final SparkMax sparkLeft =
      new SparkMax(CAN_IDs.FLYWHEEL_MOTOR_LEFT, MotorType.kBrushless);
  private final SparkMax sparkRight =
      new SparkMax(CAN_IDs.FLYWHEEL_MOTOR_RIGHT, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(0.0125, 0, 0)
          // .withTrapezoidalProfile(RPM.of(5000), DegreesPerSecondPerSecond.of(20000))
          .withSimClosedLoopController(.2, 0, 0)
          .withFeedforward(new SimpleMotorFeedforward(0.124, .124, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, .124, 0))
          // Telemetry name and verbosity level

          .withGearing(new MechanismGearing(GearBox.fromStages("1:1")))

          // Motor properties to prevent over currenting.
          .withMotorInverted(true)
          .withIdleMode(MotorMode.COAST)
          // .withStatorCurrentLimit(CurrentLimits.SHOOTER_STATOR)
          // .withSupplyCurrentLimit(Amps.of(30))
          .withTelemetry("FlyWheelMotors", TelemetryVerbosity.LOW)
          .withVoltageCompensation(Volts.of(12))
          .withFollowers(Pair.of(sparkRight, true))
          .withVendorConfig(
              new SparkMaxConfig()
                  .apply(
                      new EncoderConfig()
                          .uvwMeasurementPeriod(8)
                          .quadratureAverageDepth(2)
                          .quadratureMeasurementPeriod(8)));

  private final SmartMotorController motor =
      new SparkWrapper(sparkLeft, DCMotor.getNEO(2), smcConfig);

  private final FlyWheelConfig flywheelConfig =
      new FlyWheelConfig()
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          // .withMass(Pounds.of(2.5))
          // Maximum speed of the shooter.
          // .withUpperSoftLimit(RPM.of(5500))
          // .withLowerSoftLimit(RPM.of(0))

          // Telemetry name and verbosity for the shooter.
          .withTelemetry("FlyWheelMech", TelemetryVerbosity.LOW);

  private final FlyWheel flywheel = new FlyWheel(flywheelConfig, motor);

  /* Creates new Shooter */
  public Shooter() {
    // Idle mode is burned to flash via REV Hardware Client — verify it here at startup.
    // If this warning fires, reconnect the motor to the REV client and set coast mode, then burn.
    if (sparkRight.configAccessor.getIdleMode() != IdleMode.kCoast) {
      DriverStation.reportWarning(
          "[Shooter] WARNING: Right flywheel follower idle mode is not Coast! Burn with REV Hardware Client.",
          false);
      System.err.println(
          "[Shooter] WARNING: Right flywheel follower idle mode is not Coast! Burn with REV Hardware Client.");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flywheel.updateTelemetry();
    SmartDashboard.putNumber("Flywheel Velocity", flywheel.getSpeed().in(RPM));
  }

  @Override
  public void simulationPeriodic() {
    flywheel.simIterate();
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return flywheel.set(dutyCycle);
  }

  /** Stops the flywheel (duty cycle 0). */
  public Command stop() {
    return flywheel.set(0);
  }

  /**
   * Runs the flywheel at a fixed angular velocity setpoint using closed-loop control.
   *
   * @param targetVelocity desired wheel speed (e.g. {@code RPM.of(4000)})
   */
  public Command setVelocity(AngularVelocity targetVelocity) {
    return flywheel.run(targetVelocity);
  }

  /**
   * Runs the flywheel at a continuously-evaluated angular velocity setpoint. Useful for tracking a
   * dynamic target like {@link ShotCalculator#getIdealShooterVelocity()}.
   *
   * @param targetVelocity supplier polled every cycle
   */
  public Command setAngularVelocity(Supplier<AngularVelocity> targetVelocity) {
    return flywheel.run(targetVelocity);
  }

  /**
   * Runs the flywheel at a fixed linear (surface) velocity setpoint.
   *
   * @param targetVelocity desired surface speed
   */
  public Command setVelocity(LinearVelocity targetVelocity) {
    return flywheel.run(targetVelocity);
  }

  /**
   * Runs the flywheel at a continuously-evaluated linear (surface) velocity setpoint.
   *
   * @param targetVelocity supplier polled every cycle
   */
  public Command setLinearVelocity(Supplier<LinearVelocity> targetVelocity) {
    return flywheel.run(targetVelocity);
  }

  /**
   * Set flywheel closed loop controller to go to the specified mechanism velocity.
   *
   * @param targetVelocity Velocity to spin at.
   */
  public void setVelocitySetpoint(AngularVelocity targetVelocity) {
    flywheel.setMechanismVelocitySetpoint(targetVelocity);
  }

  /**
   * @return the current measured angular velocity of the flywheel.
   */
  public AngularVelocity getVelocity() {
    return flywheel.getSpeed();
  }

  /**
   * Returns a {@link Trigger} that is true when the flywheel is within {@code tolerance} of {@code
   * target}. Used by {@link frc.robot.commands.shootingCommands.feedWhenReady} to gate the feeder.
   */
  public Trigger isNearVelocity(AngularVelocity target, AngularVelocity tolerance) {
    return flywheel.isNear(target, tolerance);
  }

  /**
   * @return the current closed-loop velocity setpoint, or empty if running open-loop.
   */
  public Optional<AngularVelocity> getAngularVelocitySetpoint() {
    return flywheel.getMechanismSetpointVelocity();
  }

  /** Runs a SysId characterization routine (12 V max, 0.5 V/s ramp, 30 s duration). */
  
  // public Command sysId() {
  //   return flywheel.sysId(
  //       Volts.of(12), // Max voltage to apply during the test
  //       Volts.per(Second).of(0.5), // Step voltage per second
  //       Seconds.of(30) // Duration of the test
  //       );
  // }

  /**
   * Steps the flywheel through 1000-2000-3000-4000-3000-2000-1000-0 RPM, holding each step for 5
   * seconds. Useful for validating closed-loop tracking across the operating range.
   */
  public Command runVelocityStepTest() {
    final double STEP_DURATION = 5;
    return flywheel
        .run(RPM.of(1000))
        .withTimeout(STEP_DURATION)
        .andThen(flywheel.run(RPM.of(2000)).withTimeout(STEP_DURATION))
        .andThen(flywheel.run(RPM.of(3000)).withTimeout(STEP_DURATION))
        .andThen(flywheel.run(RPM.of(4000)).withTimeout(STEP_DURATION))
        .andThen(flywheel.run(RPM.of(3000)).withTimeout(STEP_DURATION))
        .andThen(flywheel.run(RPM.of(2000)).withTimeout(STEP_DURATION))
        .andThen(flywheel.run(RPM.of(1000)).withTimeout(STEP_DURATION))
        .andThen(flywheel.run(RPM.of(0)));
  }
}
