package frc.robot.DriveFiles;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;


public class DriveCommand extends Command{
    

    
    private final SwerveSubsystem s_Swerve;
    private final LimelightSubsystem s_limelight;
    public final CommandXboxController opController;
    // public final CommandJoystick leftStick;
    // public final CommandJoystick rightStick;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean fieldOriented=false;
     public double ySpeed, xSpeed, turningSpeed;
     public double ll_zSpeed, ll_xSpeed, ll_turningSpeed;
    public ChassisSpeeds chassisSpeeds;

    



    // public DriveCommand(s_Swerve s_Swerve, CommandXboxController opController, CommandJoystick leftStick, CommandJoystick rightStick) {
        public DriveCommand(SwerveSubsystem s_Swerve, CommandXboxController opController, LimelightSubsystem s_limelight) {

                this.s_Swerve = s_Swerve;
                this.s_limelight = s_limelight;
                this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.turningLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
                addRequirements(s_Swerve);
                this.opController = opController;
                // this.leftStick = leftStick;
                // this.rightStick = rightStick;
    }


    @Override
    public void initialize() {
     s_Swerve.faceAllFoward();
    }

 


    @Override
    public void execute() {
      
        // xSpeed = IsJoyStick? -leftStick.getX(): -opController.getLeftX();
        // ySpeed = IsJoyStick? -leftStick.getY(): -opController.getLeftY();
        // turningSpeed = IsJoyStick? -rightStick.getX(): -opController.getRightX();
        xSpeed = -opController.getLeftX();
        ySpeed = -opController.getLeftY();
        turningSpeed = -opController.getRightX();
        fieldOriented = s_Swerve.fieldOriented;


        
        SmartDashboard.putBoolean("fieldOriented", fieldOriented);


        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds;
        
        
        if(s_limelight.autoDriveToggle && s_limelight.hasTargets)
        {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(s_limelight.ySpeed, xSpeed, s_limelight.turningSpeed, s_Swerve.geRotation2d());
        }else
        {
            if(fieldOriented)
            {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, turningSpeed, s_Swerve.geRotation2d());
            }else
            {
                chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);
            }
        }
        s_Swerve.setModuleStates(chassisSpeeds);
    }


    @Override
    public void end(boolean interrupted) {
        s_Swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }



}
