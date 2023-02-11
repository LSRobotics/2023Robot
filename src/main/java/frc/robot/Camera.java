package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

public class Camera {
    static UsbCamera cam0;
    static public void startCameras() {
        cam0 = CameraServer.startAutomaticCapture(0);
        cam0.setResolution(240,160);
        cam0.setFPS(10);
    }
}
