// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Subsystems.ExampleSubsystem;
import frc.robot.Util.Constants.DriveConstants;
import frc.robot.Util.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Drive.Swerve;
import frc.robot.Util.Controller;
import frc.robot.Subsystems.ExampleSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public ExampleSubsystem m_ExampleSubsystem;
  public Controller u_Controller;
  public Swerve s_Swerve;
  public Drive c_Drive;
  




  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureFiles();
    configureBindings();
    s_Swerve.setDefaultCommand(c_Drive);
  }

  public SequentialCommandGroup intakeRumble()
  {
    return new SequentialCommandGroup(Commands.runOnce(
    ()->{
      u_Controller.leftStick.setRumble(RumbleType.kBothRumble, 0.5);
      u_Controller.rightStick.setRumble(RumbleType.kBothRumble, 0.5);
    }).andThen(
    Commands.waitSeconds(2.5)).andThen(
    Commands.runOnce(()->{
      u_Controller.leftStick.setRumble(RumbleType.kBothRumble, 0);
      u_Controller.rightStick.setRumble(RumbleType.kBothRumble, 0);
    })));
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    u_Controller.rightStick.button(2).toggleOnTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));
    u_Controller.rightStick.button(3).toggleOnTrue(s_Swerve.fieldOrientedToggle());
    u_Controller.rightStick.button(4).onTrue(s_Swerve.resetWheels()); //window looking button

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  public void configureFiles()
  {
    u_Controller = new Controller();
    s_Swerve = new Swerve();
    c_Drive = new Drive(s_Swerve, u_Controller.leftStick, u_Controller.rightStick);

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_ExampleSubsystem);
  }
}

