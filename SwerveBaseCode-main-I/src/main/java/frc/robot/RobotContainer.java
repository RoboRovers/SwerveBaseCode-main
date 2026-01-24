// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.DriveFiles.AutoCommand;
import frc.robot.DriveFiles.DriveCommand;
import frc.robot.DriveFiles.LimelightSubsystem;
import frc.robot.DriveFiles.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/**
 * This class is where the bulk
 *  of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController opController = new CommandXboxController(OIConstants.kOPControllerPort);
  // private final CommandJoystick leftStick = new CommandJoystick(OIConstants.kLeftStickPort);
  // private final CommandJoystick rightStick = new CommandJoystick(OIConstants.kRightStickPort);

  public static Robot robot = new Robot();
  public SwerveSubsystem s_Swerve = new SwerveSubsystem();
  public LimelightHelpers h_Limelight = new LimelightHelpers();
  public LimelightSubsystem s_Limelight = new LimelightSubsystem(s_Swerve);
  public DriveCommand d_Command = new DriveCommand(s_Swerve, opController, s_Limelight);
  public AutoCommand c_Auto = new AutoCommand(d_Command, s_Swerve, s_Limelight);

  private SendableChooser<Command> autoChooser;


 
  public RobotContainer() 
  {
    autoChooser = AutoBuilder.buildAutoChooser();
    s_Swerve.setDefaultCommand(d_Command);
    configureBindings();
    configureAuto();

    SmartDashboard.putData("Auto Chooser", autoChooser);   
  }

  public void configureAuto()
  {
    autoChooser.addOption("AutoDrive", new PathPlannerAuto("AutoDrive"));
  }



  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


  private void configureBindings() {
    //Drive Controls
    opController.povRight().toggleOnTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));
    opController.povLeft().toggleOnTrue(s_Swerve.fieldOrientedToggle());
    opController.button(7).onTrue(s_Swerve.resetWheels()); //window looking button
  }
}
