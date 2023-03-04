// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

// Imports for CANSparkMax library
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends ProfiledPIDSubsystem {
  //private final PWMSparkMax m_motor = new PWMSparkMax(ArmConstants.kMotorPort);

  // Add CAN SparkMAx
  private final CANSparkMax m_motor = new CANSparkMax(
    Constants.ArmConstants.kMotorPort,
    CANSparkMaxLowLevel.MotorType.kBrushless);

  // Get encoder from the CAN SparkMax motor created above
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  // Feedforward used in ArmSystem to counteract gravity to move arm
  // ArmSystems that are not connected to a mass will not need feedforward
  // Boolean is to control if feedfoward is used in UseOutput method
  private boolean m_useFeedForword;

  /** Create a new ArmSubsystem. */
  public ArmSubsystem(boolean useFeedForward) {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationRadPerSecSquared)),
        0);
        

    // ProfiledPIDControler constructor calls setGoal with initialPostion
    // Question : Should initialPositon be set to ArmConstants.kArmOffestRads in above vice
    //            calling setGoal(ArmConstants.kArmOffsetRads) below?

    // Set if feedfoward is needed for the armSystem

    m_motor.setIdleMode(IdleMode.kCoast);
    m_useFeedForword = useFeedForward;

    // Set the position conversastion a factor to return radians and not encoder ticks
    //m_encoder.setPositionConversionFactor(ArmConstants.kPositionConversionFactor);
    m_encoder.setPositionConversionFactor(1.0);
    System.out.println("encoder position factor: " + m_encoder.getPositionConversionFactor());

    // Is this needed to convert velocity?
    //m_encoder.setVelocityConversionFactor(ArmConstants.kEncoderDistancePerPulse/60);
    

    // Set the position of the motor encoder to be inital resting postion of the arm
    // Start arm at rest in neutral position
    setGoal(ArmConstants.kArmOffsetRads);
     
    // Enable the arm at the start
    //enable();
    System.out.println("Is Enabled? : " + this.isEnabled());
    
    
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    SmartDashboard.putNumber("Motor output", output);
    SmartDashboard.putNumber("Motor feedforward", feedforward);
    SmartDashboard.putNumber("Encoder position", m_encoder.getPosition());

    if (m_useFeedForword) {
      m_motor.setVoltage(output + feedforward);
    } else {
      m_motor.setVoltage(output);

    }

  }

  @Override
  public double getMeasurement() {
    return m_encoder.getPosition() + ArmConstants.kArmOffsetRads;
    //return (m_encoder.getPosition() + ArmConstants.kArmOffsetRads) / ArmConstants.kPositionConversionFactor  ;
  }

  public void updateSmartDash() {
    SmartDashboard.putNumber("Encoder Postion", m_encoder.getPosition());
    SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());

  }
}
