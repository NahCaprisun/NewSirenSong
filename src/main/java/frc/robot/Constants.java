// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DrivetrainConstants {
    //according to CAN IDs
    public static final int frontLeftMotor = 2;
    public static final int frontRightMotor = 1;
    public static final int backLeftMotor = 3;
    public static final int backRightMotor = 4;

    //idk where this number came from but it's been the same for the past 2 years
    public static final double rampRate = 0.5;
    public static double kDistancePerWheelRevolutionMeters = Units.inchesToMeters(6*Math.PI);
    public static double kGearReduction = 10.71;
    public static double ksVolts = 0.15239;
    public static double kvVoltSecondsPerMeter = 2.7742;
    public static double kaVoltSecondsSquaredPerMeter = 0.64165;
    public static double kPDriveVel = 0;
    public static DifferentialDriveKinematics kDriveKinematics=new DifferentialDriveKinematics(Units.inchesToMeters(22));
    public static double kRamseteB = 2.0;
    public static double kRamseteZeta = 0.7;

  
    public static double balancePowerMin = 0.1;
    public static double balancePowerMax = 0.35;
    public static double balanceDistance = 1.65;
    public static double balanceDistanceShort = 1.3;
    public static double autoTurnPower = 0.3;
    public static double mobilityDistance = 3.8;
  }
  public static final class ClimberConstants{
    public static final int rightClawMotor = 5;
    public static final double rightClawMotorPower = .4;
    public static final int leftClawMotor = 6;
    public static final double leftClawMotorPower = .4;
    
    //figure out how to get the limits set up
    public static final float rightClawLimitIn = -190f;//figure ths out
    public static final float rightClawLimitOut = 190f; // this will be negated in the subsystem because negative means out
    public static final float leftClawLimitIn = -190f; //figure this out too
    public static final float leftClawLimitOut = 190f;
  }

  public static final class IntakeConstants {
    public static final int intakeBarMotor = 7;
    public static final double intakeBarMotorPower = 0.3;
    public static final int intakeBeltMotor = 8;
    public static final double intakeBeltMotorPower = 0.3;
    public static final int trapRollerMotor = 9;
    public static final double trapRollerMotorPower = 0.3;
    public static final int switchPort = 0; 
    //public static final int pivotMotor = 6;
    //public static final double pivotMotorPower = 0.2;
    //public static final int armMotor = 7;
    //public static final double armMotorPower = 0.2;

    public static final float pivotLimitIn = 0.0f;
    public static final float pivotLimitOut = 28.0f; // this will be negated in the subsystem because negative means out
    public static final float armLimitIn = 0.0f;
    public static final float armLimitOut = 9.5f; // this will remain positive in the subsystem because positive means out
  }

  public static final class JoystickConstants {
        //Controllers
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
            
        //XboxOne Joysticks (axes)
        public static final int LEFT_STICK_X = 0;
        public static final int LEFT_STICK_Y = 1;
        public static final int RIGHT_STICK_X = 4;
        public static final int RIGHT_STICK_Y = 5;

        public static final double deadband = 0.1;
        

  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
