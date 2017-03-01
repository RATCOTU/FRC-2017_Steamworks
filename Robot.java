package org.usfirst.frc.team6236.robot;


import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;


public class Robot extends SampleRobot {
	private Joystick leftStick;
    private Joystick rightStick;
    private VictorSP left;
    private VictorSP right;
    private VictorSP climber;
    private double threshold = 0.2;
	private ADXRS450_Gyro gyro;
	private Solenoid signalLight;
	private SendableChooser<String> autoSelect;
	private Timer methodTimer;
    
	//Initialization of hardware components
    public Robot() {
        leftStick = new Joystick(0);
        rightStick = new Joystick(1);
        left = new VictorSP(0);
        right = new VictorSP(1);
        climber = new VictorSP(2);
        signalLight = new Solenoid(0);
        gyro = new ADXRS450_Gyro();
    }
    
    
    
    public void robotInit() {
    	//Autonomous selector
    	autoSelect = new SendableChooser<String>();
    	autoSelect.addDefault("Default", "Default");
    	autoSelect.addObject("Left", "Left");
    	autoSelect.addObject("Center", "Center");
    	autoSelect.addObject("Right", "Right");
    	SmartDashboard.putData("Auto Select", autoSelect);
    	
        gyro.calibrate();//Reset Gyro

    	/*
    	UsbCamera front = CameraServer.getInstance().startAutomaticCapture();//Front Camera
		front.setResolution(320, 240);
		
		UsbCamera back = CameraServer.getInstance().startAutomaticCapture();//Back Camera
		back.setResolution(320, 240);
    	
    	
    	new Thread(() -> {
    	 
    		front.setResolution(640, 480);
    		
    		CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
            
            Mat source = new Mat();
            Mat output = new Mat();
            
            while(!Thread.interrupted()) {
                cvSink.grabFrame(source);
                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                outputStream.putFrame(output);
            }            
    	}).start();
    	
    	*/
    }
    
    public void autonomous(){
    	String auto = (String) autoSelect.getSelected();//Get selected auto
    	gyro.calibrate();//Reset Gyro
    	Timer.delay(2);
    	
    	//Timer.delay(1);
    	
		switch(auto){
			case "Left":
				PIDStraight(1.7);
				gyroTurn(-45, 0);
				PIDStraight(1.5);
			break;
			case "Center":
				PIDStraight(1.7);
			break;
			case "Right":
				PIDStraight(1.7);
				gyroTurn(45, 2);
				PIDStraight(1.5);
			break;	
			case "Default":
			default:
				signalLight.set(true);
				gyroTurn(45, 2);
			break;
		}
    }

    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
        	double leftY = -leftStick.getY();//Left is reversed
        	double rightY = rightStick.getY();//Right is normal
        	
        	if(Math.abs(leftY) > threshold){//Left speed with dead zone
        		left.set(leftY);
        	}
        	else{
        		left.set(0.0);
        	}
        	
        	if(Math.abs(rightY) > threshold){//Right speed with dead zone
        		right.set(rightY);
        	}
        	else{
        		right.set(0.0);
        	}
        	
        	        			
        	if(leftStick.getTrigger() || rightStick.getTrigger()){//Climber slow forward
        		climber.set(-0.5);
        	}
        	else if(leftStick.getRawButton(2) || rightStick.getRawButton(2)){//Climber slow backwards
        		climber.set(0.5);
        	}
        	else{
        		climber.set(0.0);
        	}
        	
        	if(leftStick.getRawButton(5) || rightStick.getRawButton(5)){//Climber fast forward
        		climber.set(-1.0);
        	}
        	else if(leftStick.getRawButton(6) || rightStick.getRawButton(6)){//Climber fast backwards
        		climber.set(1.0);
        	}
        	else{
        		climber.set(0.0);
        	}
        	
        	if(leftStick.getRawButton(3) || rightStick.getRawButton(3)){//Signal light on
        		signalLight.set(true);
        	}
        	else{
        		signalLight.set(false);
        	}
        	
        	System.out.println("Gyro angle " + gyro.getAngle());
        	
        	SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        	SmartDashboard.putNumber("Gyro Rate", gyro.getRate());
        }
    }
    
    public void setSpeed(double speed_L, double speed_R){//two input speed control
	   	left.set(-speed_L);
	   	right.set(speed_R);
   	}
   
   	public void setSpeed(double speed){//one input speed control
	   	setSpeed(speed, speed);
   	}
   
   	public void PIDStraight(double timeSecs){
	   	methodTimer = new Timer();//Timer for keeping track of time
	   	methodTimer.start();//Start timer
	   	
	   	MiniPID PID = new MiniPID(0.1, 0, 100);//Adding any integral at all causes heavy oscillation
   		
	   	double goal = gyro.getAngle();//Set goal to current angle
	   	
	   	while(methodTimer.get() < timeSecs){//while timer is less than time to move
   			double error = PID.getOutput(gyro.getAngle(), goal);//Error from PID
   			
   			DriverStation.reportError("Straight Loop is running", false);
   			
   			if(error < 0){//PID controls
   				setSpeed(-0.4, -0.5);
   			}
   			else if(error > 0){
   				setSpeed(-0.5, -0.4);
   			}
   			else if(error == 0){
   				setSpeed(0);
  				DriverStation.reportError("Straight PID error is 0", false);
   			}
   			else{//For when things are broken
  				DriverStation.reportError(" Straight PID error is somehow NaN", false);
  			}
   		
   			SmartDashboard.putNumber("PID Target", goal);//Graphs
   			SmartDashboard.putNumber("PID Value", gyro.getAngle());
   			SmartDashboard.putNumber("PID error", error);
	   	}
	   	setSpeed(0);
	   	methodTimer.stop();
   	}
   	
   	public void gyroTurn(double angle, double timeout){
   		methodTimer = new Timer();//Timer for keeping track of time
	   	methodTimer.start();//Start timer
   		
	   	double goal = gyro.getAngle() + angle;
	   	
	   	while(gyro.getAngle() != goal && methodTimer.get() < timeout){
	   		if(gyro.getAngle() < goal){
	   			setSpeed(-0.5, 0.5);
	   		}
	   		else if(gyro.getAngle() > goal){
	   			setSpeed(0.5, -0.5);
	   		}
	   		else{
	   			setSpeed(0);
	   		}
	   	}
	   	
	   	setSpeed(0);
	   	methodTimer.stop();
   	}
}