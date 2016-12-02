
package org.usfirst.frc.team1846.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    String autoSelected;
    SendableChooser chooser;
    
    VictorSP leftF, leftR, rightF, rightR;
    RobotDrive drive;
    Joystick stick;
    ADXRS450_Gyro gyro;
    AnalogInput analogEncL, analogEncR;
    AnalogTrigger analogTrigL, analogTrigR;
    Encoder encL, encR, grayEnc;
    Servo camX, camY;
    
    Vision vis;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {    	
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);
        
        camX = new Servo(8);
        camY = new Servo(9);
        vis = new Vision("cam2");
        
        leftF = new VictorSP(2);
        leftR = new VictorSP(0);
        rightF = new VictorSP(3);
        rightR = new VictorSP(1);
        drive = new RobotDrive(leftR, leftF, rightR, rightF);
        drive.setExpiration(0.1);
        
        stick = new Joystick(0);
        
        analogEncL = new AnalogInput(0);
        analogEncR = new AnalogInput(1);
        analogTrigL = new AnalogTrigger(analogEncL);
        //analogTrigL.setLimitsRaw(lower, upper);
        analogTrigR = new AnalogTrigger(analogEncR);
        encL = new Encoder(0, 1, true, CounterBase.EncodingType.k4X);
        encR = new Encoder(2, 3, false, CounterBase.EncodingType.k4X);
        grayEnc = new Encoder(4, 5, false, CounterBase.EncodingType.k4X);
        gyro = new ADXRS450_Gyro();
        gyro.calibrate();
        
        LiveWindow.addActuator("Drive", "Left Front", leftF);
        LiveWindow.addActuator("Drive", "Left Rear", leftR);
        LiveWindow.addActuator("Drive", "Right Front", rightF);
        LiveWindow.addActuator("Drive", "Right Rear", rightR);
        LiveWindow.addSensor("Localization", "Gyro", gyro);
        
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    public void autonomousInit() {
    	gyro.reset();
    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	updateDashboard();
    	drive.drive(-0.2, -gyro.getAngle()*0.03);
    	Timer.delay(0.005);
    }
    
    public void teleopInit(){
    	gyro.reset();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        drive.arcadeDrive(stick);
        updateDashboard();
        
        Timer.delay(0.005);
    }
    
    public void testInit(){
    	gyro.reset();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	gyro.updateTable();
    	updateDashboard();
    }
    
    private void updateDashboard(){
    	SmartDashboard.putData("Left Front", leftF);
        SmartDashboard.putData("Left Rear", leftR);
        SmartDashboard.putData("Right Front", rightF);
        SmartDashboard.putData("Right Rear", rightR);
        SmartDashboard.putData("Gyro" , gyro);
        SmartDashboard.putData("Encoder Left", encL);
        SmartDashboard.putData("Encoder Right", encR);
        SmartDashboard.putData("Grayhill Encoder", grayEnc);
        SmartDashboard.putData("Analog Encoder Left", analogEncL);
        SmartDashboard.putData("Analog Encoder Right", analogEncR);
        SmartDashboard.putData("Camera X", camX);
        SmartDashboard.putData("Camera Y", camY);
    }
    
}
