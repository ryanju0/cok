// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax kickerMotor;
  private CANSparkMax pivotMotor;
  private AbsoluteEncoder pivotEncoder;


  private CANSparkMax topFlywheelMotor;
  private CANSparkMax bottomFlywheelMotor;
  private RelativeEncoder flywheelEncoder;


  private PIDController pivotController = new PIDController(0, 0, 0);
  private PIDController flywheelController = new PIDController(0.5, 0, 0);
  public static final int kKickerMotorCanId = 13;
  public static final int kPivotMotorCanId = 15;
  public static final int kKickerMotorCurrentLimit = 30;
  public static final int kPivotMotorCurrentLimit = 30;
  public static final int kKickerNominalVoltage = 12;
  public static final double kPivotPositionConversionFactor = (2*Math.PI * 50);
  public static final double kKickerIntakeMotorSpeed = 0.3;
  public static final double kKickerOuttakeMotorSpeed = -0.6;
  public static final double kPivotP = 3.0;
  public static final double kPivotGearRatio = 50;

  /** Creates a new Shooter. */
  public Shooter() {
      kickerMotor = new CANSparkMax(kKickerMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
      pivotMotor = new CANSparkMax(kPivotMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
      kickerMotor.setInverted(true);
      pivotMotor.setInverted(true);
      kickerMotor.setIdleMode(IdleMode.kBrake);
      pivotMotor.setIdleMode(IdleMode.kBrake);
      topFlywheelMotor.setIdleMode(IdleMode.kCoast);
      bottomFlywheelMotor.setIdleMode(IdleMode.kCoast);
      flywheelEncoder = topFlywheelMotor.getEncoder();
      kickerMotor.setSmartCurrentLimit(kKickerMotorCurrentLimit);
      pivotMotor.setSmartCurrentLimit(kPivotMotorCurrentLimit);
      kickerMotor.enableVoltageCompensation(kKickerNominalVoltage);
      pivotMotor.enableVoltageCompensation(kKickerNominalVoltage);
      pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
      pivotEncoder.setPositionConversionFactor(kPivotPositionConversionFactor);
      bottomFlywheelMotor.follow(topFlywheelMotor, true);
      pivotController.disableContinuousInput();
      pivotController.setTolerance(Units.degreesToRadians(7));
  }
  public double getPivotAngleRadians() {
    return (pivotEncoder.getPosition() / kPivotGearRatio);
  }
  public double getFlywheelVelocity() {
    return flywheelEncoder.getVelocity();
  }
  public void setKickerIntake(double power) {
    kickerMotor.setVoltage(kKickerNominalVoltage);
  }
  public Command setPreset(double PivotDegrees, double FlywheelRPM) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        flywheelController.setSetpoint(Units.rotationsPerMinuteToRadiansPerSecond(FlywheelRPM));
        pivotController.setP(kPivotP); //prevent jumping on enable p = 2.5
        pivotController.setSetpoint(Units.degreesToRadians(PivotDegrees));
      })
    );
  }
  public Command intakeSequence() {
    return new ParallelCommandGroup(
      new InstantCommand(() -> setKickerIntake(kKickerIntakeMotorSpeed)),
      setPreset(115, -100));
  }
  public Command outtakeSequence() {
    return new ParallelCommandGroup(
      new InstantCommand(() -> setKickerIntake(kKickerOuttakeMotorSpeed)),
      setPreset(90, 50));
  }

  @Override
  public void periodic() {
    pivotMotor.setVoltage(pivotController.calculate(getPivotAngleRadians()));
    topFlywheelMotor.setVoltage(flywheelController.calculate(getFlywheelVelocity()));
    // This method will be called once per scheduler run
  }
}
