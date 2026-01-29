package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class Controller {
    public CommandJoystick leftStick, rightStick;

    public Controller()
    {
        leftStick = new CommandJoystick(0); //Change this
        rightStick = new CommandJoystick(0); //Change this
    }
}
