// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BackPhotonVision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LeftLimelight;
import frc.robot.subsystems.PigeonAutoAim;
//import frc.robot.subsystems.LeftPhotonVision;
import frc.robot.subsystems.RightLimelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.BackPhotonVision;
//mine:
import frc.robot.Constants;
import frc.robot.commands.AddDegree;
import frc.robot.commands.AmpShotArm;
import frc.robot.commands.AutoAim;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.NewAutoAim;
import frc.robot.commands.NoteRstCmd;
import frc.robot.commands.ReleaseCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.ShooterSpoolCmd;
import frc.robot.commands.ZeroArmPos;
import frc.robot.commands.RemoveDegree;




public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Indexer m_indexer = new Indexer();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Arm m_arm = new Arm();
  private final BackPhotonVision m_backPhotonVision = new BackPhotonVision();
  //private final LeftPhotonVision m_leftPhotonVision = new LeftPhotonVision();
  private final RightLimelight m_rLimelight = new RightLimelight();
  private final LeftLimelight m_lLimelight = new LeftLimelight();
  private final PigeonAutoAim m_PigeonAutoAim = new PigeonAutoAim();
  







  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double  MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain



  // deprecated as of 4-20  //private final GenericHID drivController = new GenericHID(0); // BENS CONTROLLER

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * (0.125)*0.25).withRotationalDeadband(MaxAngularRate * (0.1)*1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    /* deprecated 4-20 new JoystickButton(drivController, 1).toggleOnTrue(new IntakeCmd(m_intake, m_indexer)); // runs the intake command on b (1) button on xbox controller
    new JoystickButton(drivController, 0).toggleOnTrue(new ShootCmd(m_shooter, m_indexer));
    new JoystickButton(drivController, 2).onTrue(new NoteRstCmd(m_indexer, m_intake));*/

    //Ben's Commands
    joystick.rightBumper().toggleOnTrue(new IntakeCmd(m_intake, m_indexer, m_shooter)); //on right bumper button run intake
    joystick.x().toggleOnTrue(new ShootCmd(m_shooter, m_indexer, m_arm)); //on right trigger button shoot auto
    joystick.y().onTrue(new NoteRstCmd(m_indexer, m_intake)); // on y button back out indexer manual **do not need this normally
    //joystick.leftTrigger().toggleOnTrue(new ShooterSpoolCmd(m_shooter));
    //joystick.rightTrigger(0.75).toggleOnTrue(new ReleaseCmd(m_indexer));
    //joystick.pov(180).onTrue(new RemoveDegree(m_arm));
    //joystick.pov(0).onTrue(new AddDegree(m_arm));
    joystick.pov(0).onTrue(new AmpShotArm(m_arm));
    joystick.pov(180).onTrue(new ZeroArmPos(m_arm));
    joystick.pov(90).toggleOnTrue(new AutoAim(m_arm, m_backPhotonVision, drivetrain, m_shooter, m_indexer, m_lLimelight, m_rLimelight, m_PigeonAutoAim));
    joystick.pov(270).toggleOnTrue(new NewAutoAim(drivetrain, m_backPhotonVision));

     



    
    
    
    
    
    //swervestuff
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX((-joystick.getLeftY() * MaxSpeed)*0.25) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY((-joystick.getLeftX() * MaxSpeed)*0.25) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    joystick.leftStick().toggleOnTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    drivetrain.registerTelemetry(logger::telemeterize);

    //joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    //joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));


    /* Bindings for drivetrain characterization */
    
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  }

  public RobotContainer() {
    configureBindings();
    NamedCommands.registerCommand("Intake", new IntakeCmd(m_intake, m_indexer, m_shooter));
    NamedCommands.registerCommand("Shoot", new ShootCmd(m_shooter, m_indexer, m_arm));
    NamedCommands.registerCommand("wait", new WaitCommand(0.7));
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    //return runAuto;
    drivetrain.runOnce(() -> drivetrain.seedFieldRelative());

    
    return new PathPlannerAuto("Auto Center");
  }
}
