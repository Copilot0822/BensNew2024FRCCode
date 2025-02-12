// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }






  //CAN Ids
  public static final int intakeId = 47;
  public static final int indexerId = 40;
  public static final int bottomShooterId = 41;
  public static final int topShooterId = 42;
  public static final int leftArmId = 43;
  public static final int rightArmId = 44;

   //Motor Inverts
  public static final boolean intakeInvert = true;
  public static final boolean indexerInvert = true;
  public static final boolean shooterTopInvert = true;
  public static final boolean shooterBottomInvert = true;
  
  //Intake Command vars
  public static final double intakeSpeedIn = 0.8;
  public static final double indexSpeedIn = 0.5;
  public static final double indexCurrentThreshould = 10;
  public static final int indexCurrentDelay = 400;
  
  public static final double indexBackOutSpeed = 0.4;
  public static final int indexBackOutTime = 100;

  //Shoot Command vars
  public static final double shooterSpeed = 1;
  public static final double shooterVariance = 50; //rpm variance
  public static final double indexerPush = 1;
  public static final int afterIndexPushDelay = 1500;
  public static final int shooterRPMLim = 5100;


  //arm vars
  public static final double armSlope = 0.063;
  public static final double downArmSlope = 0.03125;
  public static final double maxUpAppliedPower = 0.4;
  public static final double maxDownAppliedPower = -0.2;





  // joystick curve
  public static final double joystickSlope = 0.5;
  public static final double speedMultiplier = 0.25;
  public static final double deadzoneLeft = 0.125;
  public static final double deadzoneDrive = joystickSlope*(deadzoneLeft - 1)*deadzoneLeft*(deadzoneLeft + 1) + deadzoneLeft;
  


  //add degree vars
  public static final int AddDegreeAmount = 5;
  public static final int RemoveDegreeAmount = 1;


  //amp shot vars
  public static final double ampArmPos = 70;
  

  public static class OperatorConstants {
    //public static final int kDriverControllerPort = 0;
  }
}
