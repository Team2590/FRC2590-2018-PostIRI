package org.usfirst.frc.team2590.usbVision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team2590.robot.Robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

/**
 * <h1> VISION </h1>
 *
 * Main handler class to alter values for the robots cameras
 * Currently does support multiple usb cameras
 *
 * @version 1.0
 * @author Connor_Hofenbitzer
 *
 */
public class CameraHandler {

	//a usb camera instance for vision processing
	private UsbCamera camera;
	private CameraPresets presets;

	public CameraHandler(int deviceNum, CameraPresets camPresets) {
	  //init stuff
	  presets = camPresets;
		camera = CameraServer.getInstance().startAutomaticCapture(0);
		
		//setDefaults();
	}

	@SuppressWarnings("static-access")
  public void setDefaults() {
	  setFPS(CameraPresets.FPS);
    setExposure(CameraPresets.exposure);
    setBrightness(CameraPresets.brightness);
    setWhiteBalance(CameraPresets.whiteBalance);
    setResolution(CameraPresets.height, CameraPresets.width);
	}

	
	/**
	 * Sets the cameras exposure
	 * @param exposure : exposure to set to
	 */
	public void setExposure(int exposure) {
		camera.setExposureManual(exposure);
	}

	/**
	 * Sets the cameras brightness
	 * @param brightness : brightness to set to
	 */
	public void setBrightness(int brightness) {
		camera.setBrightness(brightness);
	}

	/**
	 * Sets the cameras white balance
	 * @param whiteBal : white balance to set the camera to
	 */
	public void setWhiteBalance(int whiteBal) {
		camera.setWhiteBalanceManual(whiteBal);
	}

	/**
	 * Sets the cameras resolution
	 * @param height : cameras height in pixels
	 * @param width  : cameras width  in pixels
	 */
	public void setResolution(int height, int width) {
		camera.setResolution(width, height);
	}

	/**
	 * Sets the cameras frames per second
	 * @param FPS : frames per second
	 */
	public void setFPS(int FPS) {
		camera.setFPS(FPS);
	}

	/**
	 * gets the cameras current connection state
	 * @return : if the camera is connected
	 */
	public boolean getConnected() {
		return camera.isConnected();
	}

}
