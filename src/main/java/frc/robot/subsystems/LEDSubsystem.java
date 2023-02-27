package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase{
    private Spark LED = new Spark(Constants.LEDConstants.ledControllerID);
    
    public LEDSubsystem(){
        super();
    }

    public void setMode(boolean cubeMode){
        if(cubeMode){
            LED.set(Constants.LEDConstants.cubeColour);
        } else {
            LED.set(Constants.LEDConstants.coneColor);  
        }
    }

}

