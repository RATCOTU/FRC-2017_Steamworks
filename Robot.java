package org.usfirst.frc.team6236.robot;


import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;


public class Robot extends SampleRobot {
	private Joystick leftStick;
    private Joystick rightStick;
    private final String defaultAuto = "Default";
    private final String testAuto = "Test Auto";
    private VictorSP left;
    private VictorSP right;
    private VictorSP climber;
    private double threshold = 0.1;
	private VisionThread visionThread;
	/*
	private ADXL345_I2C_SparkFun m_accel;
	private GyroITG3200 m_gyro;
	*/
	private ITG3200 m_gyro;
	
	//Lacey is da best out of all of the people in da world
	//But she is also incompetent at grammar
	//Her boyfriend though(who is the creator and maintainer of this code) smart guy who is also not ugly and reasonably attractive
	//Also really fukken tall
	private final Object imgLock = new Object();
    
    public Robot() {
        leftStick = new Joystick(0);
        rightStick = new Joystick(1);
        left = new VictorSP(0);
        right = new VictorSP(1);
        climber = new VictorSP(2);
    }
    
    
    
    public void robotInit() {
    	/*
    	m_accel = new ADXL345_I2C_SparkFun(I2C.Port.kOnboard, Accelerometer.Range.k16G);

    	m_gyro = new GyroITG3200(I2C.Port.kOnboard);
    	m_gyro.initialize();
    	*/
    	
    	m_gyro = new ITG3200(I2C.Port.kOnboard, false);
    	
    	new Thread(() -> {
    		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    		camera.setResolution(640, 480);
    		
    		CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
            
            Mat source = new Mat();
            Mat output = new Mat();
            
            while(!Thread.interrupted()) {
                cvSink.grabFrame(source);
                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                outputStream.putFrame(output);
            }
    		
            /* 
             visionThread = new VisionThread(camera, new GripPipeline(), pipeline ->{
    			if(!pipeline.maskOutput().empty()){
    				synchronized (imgLock) {
        			
    				}
    			}
    		});
    		visionThread.start();
             */
            
    	}).start();
    }

    public void autonomous(){
    	MiniPID straightPID = new MiniPID(0, 0, 0, 0);
    	straightPID.setPID(1.0, 1.0, 1.0);
    	
    	double goal = m_gyro.getZ();
    	
    	straightPID.getOutput(m_gyro.getZ(), goal);
    }

    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
        	double leftY = -leftStick.getY();
        	double rightY = rightStick.getY();
        	
        	if(Math.abs(leftY) > threshold){//Speed left
        		left.set(leftY);
        	}
        	else{
        		left.set(0.0);
        	}
        	
        	if(Math.abs(rightY) > threshold){//Speed right
        		right.set(rightY);
        	}
        	else{
        		right.set(0.0);
        	}
        	
        	if(leftStick.getRawButton(1) || rightStick.getRawButton(1)){
        		climber.set(1.0);
        	}
        	else if(leftStick.getRawButton(2) || rightStick.getRawButton(2)){
        		climber.set(-1.0);
        	}
        	else{
        		climber.set(0.0);
        	}
        	
        	
        	DriverStation.reportError("Joystick left" + leftY, false);
        	DriverStation.reportError("Joystick right" + rightY, false);
        	
        	SmartDashboard.putNumber("Joystick left", leftY);
        	SmartDashboard.putNumber("Joystick right", rightY);

        	m_gyro.updateDashboard("Gyro 1", true);
        	m_gyro.clearDashboard("Gyro 1");
        	
        	/*
        	SmartDashboard.putNumber("Gyro X", m_gyro.getRotationX());
        	SmartDashboard.putNumber("Gyro Y", m_gyro.getRotationY());
        	SmartDashboard.putNumber("Gyro Z", m_gyro.getRotationZ());
			*/
        }
    }
    
   public void setSpeed(double speed_L, double speed_R){//two input speed control
	   left.set(-speed_L);
	   right.set(speed_R);
   }
   
   public void setSpeed(double speed){//one input speed control
	   setSpeed(speed, speed);
   }
}
