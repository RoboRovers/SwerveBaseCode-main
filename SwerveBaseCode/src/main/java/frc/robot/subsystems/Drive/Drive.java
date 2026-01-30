package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Util.Constants.DriveConstants;
import frc.robot.Subsystems.Drive.Swerve;


public class Drive extends Command{

    public final Swerve s_Swerve;
    public final CommandJoystick left, right;

    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean fieldOriented=false;
    public double ySpeed, xSpeed, turningSpeed;
    public ChassisSpeeds chassisSpeeds;

    public static final double DEADBAND = 0.075;

    



    // public DriveCommand(s_Swerve s_Swerve, CommandXboxController opController, CommandJoystick leftStick, CommandJoystick rightStick) {
    public Drive(Swerve s_Swerve, CommandJoystick left, CommandJoystick right) {

        this.s_Swerve = s_Swerve;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.left = left;
        this.right = right;
        addRequirements(s_Swerve);
    }


    @Override
    public void initialize()
    {
        s_Swerve.faceAllFoward();
    }

      @Override
    public void execute() {
        xSpeed = -right.getY();
        ySpeed = -right.getX();
        turningSpeed = -left.getX();
        fieldOriented = s_Swerve.fieldOriented;

        xSpeed = Math.abs(xSpeed) > DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > DEADBAND ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > DEADBAND ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond;

        drive();

    }
    
    public void drive()
    {
        if(fieldOriented)
        {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, turningSpeed), s_Swerve.getRotation2d());
        }else
        {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }
        s_Swerve.setModuleStates(chassisSpeeds);        
    }

    public void driveAlgae(double xSpeed, double turningSpeed)
    {
        chassisSpeeds = new ChassisSpeeds(xSpeed, this.ySpeed, turningSpeed);
        s_Swerve.setModuleStates(chassisSpeeds);
    }

    public void driveReef(double xSpeed, double ySpeed, double turningSpeed)
    {
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(ySpeed, xSpeed, turningSpeed), s_Swerve.getRotation2d());
        s_Swerve.setModuleStates(chassisSpeeds);
    }


    @Override
    public void end(boolean interrupted) {
        s_Swerve.stopModules();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }



}



 