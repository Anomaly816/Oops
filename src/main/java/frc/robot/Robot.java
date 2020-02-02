/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


// I done went and broke things so I fixed real quick, also it was your brother's fault for not having a bus pass and you losing yours because of it :p

package frc.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import frc.robot.GripPipeline;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  //Vision Test

  //----------------------------
  VisionThread visionThreadanom;
  private final Object imgLock = new Object();
  private double synctest;
  private double ballarea;
  private int ballwidth;
  private int ballheight;



  double[] xVal;
  double[] defaultValue = new double[0];
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final AnalogInput intakeSensor = new AnalogInput(0);
  private final AnalogInput outakeSensor = new AnalogInput(1);
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  public static final ADIS16448_IMU imu = new ADIS16448_IMU();
  public double kAngle = 0.0;
  public static final double kP = 0.01;
  public static final double kI = 0;
  public static final double kD = 0;
  public double prevError = 0.0;
  public double integral = 0.0;
  public double error = 0.0;
  public double derivative = 0.0;
  public double pidValue = 0.0;

  public double kAngleR = 0.0;
  public static final double rkP = 0.005;
  public static final double rkI = 0;
  public static final double rkD = 0;
  public double prevErrorR = 0.0;
  public double integralR = 0.0;
  public double errorR = 0.0;
  public double derivativeR = 0.0;
  public double pidValueR = 0.0;

  public double pidValueT = 0.0;

double leftt;
double rightt;
int bCount = 0;
int shouldCount = 1;
int shouldDCount = 1;
Joystick leftjoy;
Joystick rightjoy;
DifferentialDrive adrive;
Color prevColor;
double bluec = 0; //prev color is 1
double yellowc = 0; //prev color is 2
double redc = 0; //prev color is 3
double greenc = 0; // prev color is 4
double  totalr = 0; 
double prevc = 0; // this is because the color is checked every .5 sec so its giving mult. values even though it's not changing color
double curc = 0; // same as above exept this is for current color. 
double prevTime;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("GRIP/816Blobs");


    
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(180, 120);

    visionThreadanom = new VisionThread(camera, new GripPipeline(), pipeline ->{
     if(!pipeline.filterContoursOutput().isEmpty()){
        Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
        synchronized(imgLock){
          ballarea = r.area();
          ballwidth = r.width;
          ballheight = r.height;
          synctest = r.x + (r.width/2);
        }
      }
      
    });
    visionThreadanom.start();


    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    Talon leftt = new Talon(1);
    Talon rightt = new Talon(0);
     leftjoy = new Joystick(1);
     rightjoy =   new Joystick(0);
    adrive = new DifferentialDrive(leftt, rightt);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);   
  }
  double x = 0;
  double y = 0;
  double[] eh;
   /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    double synctestT;
    double areaball;
    int ballwidtha;
    int ballheighta;
    synchronized(imgLock){
      synctestT = this.synctest;
      areaball = this.ballarea;
      ballwidtha = this.ballwidth;
      ballheighta = this.ballheight;
    }
    SmartDashboard.putNumber("x pos", synctestT);
    
    SmartDashboard.putNumber("Ball Width", ballwidtha);
    SmartDashboard.putNumber("Ball Height", ballheighta);
    SmartDashboard.putNumber("Ball Area", ballheighta*ballwidtha);
   

    Color detectedColor = m_colorSensor.getColor();

    double IR = m_colorSensor.getIR();
    SmartDashboard.putNumber("Entry x?", x);
    SmartDashboard.putNumber("Entry y?", y);
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    int proximity = m_colorSensor.getProximity();
    double intakeSensorVal = intakeSensor.getValue();
    double outakeSensorVal = outakeSensor.getValue();
    SmartDashboard.putNumber("Intake", intakeSensorVal);    
    SmartDashboard.putNumber("Outake", outakeSensorVal);  
    SmartDashboard.putNumber("Proximity", proximity);
    
    SmartDashboard.putNumber("SensorAngle", imu.getAngle());
    SmartDashboard.putNumber("SensorAngleRate", imu.getRate());
    SmartDashboard.putNumber("PID VALUE TOTAL", pidValueT);

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    // Acosta Ball Count

    if (intakeSensorVal >= 600 && shouldCount == 1){
      bCount = bCount + 1;
      shouldCount = 0;
    }else if(intakeSensorVal <350 && shouldCount == 0){
      shouldCount = 1;
    }

    if (outakeSensorVal >= 600 && shouldDCount == 1){
      bCount = bCount - 1;
      shouldDCount = 0;
    }else if(outakeSensorVal <350 && shouldDCount == 0){
      shouldDCount = 1;
    }

    SmartDashboard.putNumber("Balls", bCount);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
      if(prevColor != match.color){
        bluec = bluec + 1;
        prevColor = match.color;
      }
     prevc = 1; 
    } else if (match.color == kRedTarget) {
      colorString = "Red";
      if(prevColor != match.color){
        redc = redc + 1;
        prevColor = match.color;
      }
     prevc = 2;
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
      if(prevColor != match.color){
        greenc = greenc + 1;
        prevColor = match.color;
      }
      prevc = 3;
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
      if(prevColor != match.color){
        yellowc = yellowc + 1;
        prevColor = match.color;
      }
      prevc = 4;
    } else {
      prevColor = null;
      colorString = "Unknown";
    }
    
    //if(prevc =)

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putNumber("BlueC",bluec);
    SmartDashboard.putNumber("redc",redc);
    SmartDashboard.putNumber("greenc",greenc);
    SmartDashboard.putNumber("yellowc",yellowc);
    SmartDashboard.putNumber("totalr",totalr);

    

    if((bluec + redc + yellowc +  greenc) / 4 >= 3){
      bluec = 0;
      yellowc = 0;
      greenc = 0;
      redc = 0;

      totalr = totalr + 1.0;
    }

    

    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    prevTime = System.currentTimeMillis();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
     leftt = 0.15;
    rightt = 0.15;
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        errorR = kAngleR - imu.getRate();
        integralR +=  (errorR*.02);
        derivativeR = (errorR - prevErrorR)/.02;
        prevErrorR = errorR;
        pidValueR = rkP*errorR + rkI*integralR + rkD*derivativeR;

        error = kAngle - imu.getAngle();
        integral +=  (error*.02);
        derivative = (error - prevError)/.02;
        prevError = error;
        pidValue = kP*error + kI*integral + kD*derivative;
        
        pidValueT = pidValue + pidValueR;
        SmartDashboard.putNumber("Time Running", System.currentTimeMillis() - prevTime);
        SmartDashboard.putNumber("prev Error", prevError);
        SmartDashboard.putNumber("PID", pidValue);
        if(System.currentTimeMillis() - prevTime < 5000){
          kAngle = 0.0;
          kAngleR = 0.0;
          adrive.tankDrive((pidValueT)+0.5, 0.5-pidValueT);
        }else if(System.currentTimeMillis() - prevTime >= 5000 && System.currentTimeMillis() - prevTime <= 10000){
          if(error > 20){
            kAngleR = 40.0;
          }else{
            kAngleR = 0.0;
          }
          
          
          kAngle = 180.0;
          adrive.tankDrive((pidValueT)+0.5, 0.5-pidValueT);
        }else{
          adrive.tankDrive(0, 0);
        }
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double Leftjoy = leftjoy.getRawAxis(1);
    double Rightjoy = rightjoy.getRawAxis(1);
    boolean slowr = rightjoy.getRawButtonPressed(3);
    boolean slowl = leftjoy.getRawButtonPressed(3);
    //boolean fastr = rightjoy.getRawButtonPressed(1);
    //boolean fastl = leftjoy.getRawButtonPressed(1);

    adrive.tankDrive(-Leftjoy *.75, -Rightjoy*.75);

    if(slowr || slowl){
      Leftjoy = leftjoy.getRawAxis(1)*.25;
      Rightjoy =rightjoy.getRawAxis(1)*.25;
    }else{
      Leftjoy = leftjoy.getRawAxis(1)*.75;
      Rightjoy =rightjoy.getRawAxis(1)*.75;

    }
  }

  public static double cubicScale (double input){
    return input*input*input;
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
