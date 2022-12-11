// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoTurret;
//import frc.robot.commands.DistanceAuto;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ProtoTurret;
import frc.robot.commands.SpinToTarget;
//import frc.robot.commands.TimedAuto;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.limelight;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  
  //We have to initialize these objects for the SpinToTarget, ProtoTurret, and AutoTurret commands
  private final static DriveTrain drive = new DriveTrain();
  private final static limelight lime = new limelight();
  private final static Turret turret = new Turret();
  private final static SpinToTarget spin = new SpinToTarget(drive, lime);
  private final static ProtoTurret proto = new ProtoTurret(drive, lime);
  private final static AutoTurret auto = new AutoTurret(lime, turret);
  
  private static Joystick joy1;
  private static Joystick joy2;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    joy1 = new Joystick(Constants.joy1);
    joy2 = new Joystick(Constants.joy2);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // We have to return the name of the object, which is spin for SpinToTarget in this case or the code will not work
    return spin;
    //return proto;
    //return auto;
  }
  public static Joystick getJoy1(){
    return joy1;
  }
  public static Joystick getJoy2(){
    return joy2;
  }
  public static DriveTrain getDrive(){
    return drive;
  }
}