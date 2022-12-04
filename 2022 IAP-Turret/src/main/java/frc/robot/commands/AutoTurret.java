// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.Turret;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutoTurret extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  public final limelight lime;
  public final Turret turret;
  public PIDController pid;
  public int limitSwitchClosed = 1;
  public int limitSwitchOpen = 0;
  public boolean manualToggle = true;
  public boolean directionToggle;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoTurret(limelight lime, Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.lime = lime;

     /*
    * P is the Porportional constant
    * It will correct the error depending on how big the amount of error is
    * Small amount of error = low correction, High = larger correction
    */
    double kP = 0.0;
    /*
    * I is the Integral constant
    * It adds up all the past errors to help remove constant errors, because
    * no matter how small the constant error is, the sum will be significant enough 
    * to adjust the controller output as needed
    */
    double kI = 0.0;
    /*
    * D is the Derivative constant
    * It will predict the amount of error in the future, because it examines
    * the slope of the change in error
    */
    double kD = 0.0;
    pid = new PIDController(kP, kI, kD); 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    turret.setAngle(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(RobotContainer.getJoy1().getTriggerReleased()){
      manualToggle = !manualToggle;
    }
    if(manualToggle == true){
      turret.spin(-0.2*RobotContainer.getJoy1().getX()); //The joystick returns -1 so we use -0.2
    }
    if(!manualToggle == true){

      if(lime.get_tv() == 1 && !(turret.getCCW_Reverse_LimitSw() == limitSwitchClosed | turret.getCW_Forward_LimitSw() == limitSwitchClosed)){
        double speed = pid.calculate(lime.get_tx());
        turret.spin(speed); //This should spin clockwise
      }

      else if(lime.get_tv() == 0){

        if(turret.getCCW_Reverse_LimitSw() == limitSwitchClosed){
          directionToggle = true;
        }

        if(turret.getCW_Forward_LimitSw() == limitSwitchClosed){
          directionToggle = false;
        }

        if(directionToggle == true){
          turret.spin(0.3); //Will go CW if the reverse limit switch is hit
        }

        if(!directionToggle == true){
          turret.spin(-0.3); //Will go CCW if the forward limit switch it hit
        }

      } else {
        turret.spin(0); //Will disable motor
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}