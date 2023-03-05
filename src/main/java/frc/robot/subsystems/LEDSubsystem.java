package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase{
    private Spark LED = new Spark(Constants.LEDConstants.ledControllerID);
    private boolean internalCubeMode;
    private boolean inBrake = false;

    public LEDSubsystem(boolean initialColor){
        super();
        this.internalCubeMode = initialColor;
        updateColor();
    }

    public void setMode(boolean cubeMode){
        internalCubeMode = cubeMode;
        updateColor();
    }
    
    public void updateColor() {
        if (inBrake) {
            LED.set(Constants.LEDConstants.brakeColor);
            return;
        }

        if(internalCubeMode){
            LED.set(Constants.LEDConstants.cubeColour);
        } else {
            LED.set(Constants.LEDConstants.coneColor);  
        }
    }

    public void setBrakeMode() {
        inBrake = true;
        updateColor();
    }

    public void disableBrakeMode() {
        inBrake = false;
        updateColor();
    }

}

