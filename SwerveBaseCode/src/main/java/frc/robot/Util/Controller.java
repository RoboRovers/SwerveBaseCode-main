package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Util.RobotMap.Map_Controller;

public class Controller {
    public CommandJoystick leftStick, rightStick;

    public Controller()
    {
        leftStick = new CommandJoystick(Map_Controller.Left_Joystick); 
        rightStick = new CommandJoystick(Map_Controller.RIght_Joystick); 
        
    }
}
