

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
//importing extra wpilib classes
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public abstract class Balancer extends SubsystemBase {

  private final PWMSparkMax balancerPWM;
  private double position;
  private double dClawMotorPower = 0;
  private double speed = 0.5;

  public Balancer(double inputSpeed, int channel) {
    balancerPWM = new PWMSparkMax(channel);
    speed = inputSpeed;

    setDefaultCommand(new RunCommand(this::stop, this));

    //set up right claw motor settings
    // clawMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // clawMotor.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.rightClawLimitIn);
    // clawMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // clawMotor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.rightClawLimitOut);
  }

  public void stop() {
    balancerPWM.stopMotor();
  }

  public void BalancerUp() {
    balancerPWM.set(-speed);
  }

  public void BalancerDown() {
    balancerPWM.set(speed);
  }

  public void SetBalancerSpeed(double inputSpeed) {
    balancerPWM.set(inputSpeed);
  }

  public double getPosition () {
    return position;
  }

  @Override
  public void periodic() {
    //figure out how to get the shuffleboard to output the position of the encoder here
    position = balancerPWM.get();
    SmartDashboard.putNumber("Position", balancerPWM.get());
    SmartDashboard.putNumber("Speed", speed);
  }
  
  /**
   * Example command factory method.
   *
   * @return a command
   
  public void setClimber(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here 

        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  */
}
