// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  // the roller that pulls balls into the mechanism
  private final SparkMax intakeRoller;
  // the separated brushless launcher motor
  private final SparkMax launcherMotor;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
  // create motors for the launcher mechanism
  // intakeRoller: brushed motor that handles the intake roller
  intakeRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);
  // feeder roller (pushes fuel away from the launcher)
  feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushed);
  // launcherMotor: separated brushless motor (NEO)
  launcherMotor = new SparkMax(INTAKE_FEEDFRONT, MotorType.kBrushless);


    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("LAUNCHER value", LAUNCHER_VOLTAGE);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(CIM_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
  SparkMaxConfig intakeConfig = new SparkMaxConfig();
  intakeConfig.smartCurrentLimit(CIM_MOTOR_CURRENT_LIMIT);
  intakeRoller.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  // create the configuration for the launcher motor, set a current limit,
  // set the motor to inverted so that positive values are used for launching,
  // and apply the config to the controller
  //SparkMaxConfig launcherConfig = new SparkMaxConfig();
  //launcherConfig.inverted(true);
  //launcherConfig.smartCurrentLimit(NEO_MOTOR_CURRENT_LIMIT);
  //launcherMotor.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederRoller.setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
  intakeRoller
    .setVoltage(SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederRoller
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
  intakeRoller
    .setVoltage(-1 * SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for launching.
  public void launch() {
  launcherMotor.setVoltage(SmartDashboard.getNumber("LAUNCHER value", LAUNCHER_VOLTAGE));
  feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
  intakeRoller
    .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  // A method to stop the rollers
  public void stop() {
    launcherMotor.set(0);
    feederRoller.set(0);
    intakeRoller.set(0);
  }

  // Run launcher motor only (useful for direct operator control)
  public void runLauncher() {
    launcherMotor.setVoltage(SmartDashboard.getNumber("LAUNCHER value", LAUNCHER_VOLTAGE));
  }

   //Stop only the launcher motor
  public void stopLauncher() {
    launcherMotor.set(0);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    feederRoller
        .setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
  launcherMotor.setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  intakeRoller
    .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launch());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
