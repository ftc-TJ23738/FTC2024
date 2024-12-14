//import all packages

/*
______  ______  ______
\    /  \    /  \    /
 \  /    \  /    \  /
  \/      \/      \/
  0        0       0
  this link is google drawing, taking you to a layout of the controls for the robot
  This robot is controlled by a LOGITECH F310 gamepad.
  https://docs.google.com/drawings/d/1TKSZY56RuRNBei_8up7gPKN2joNKI1WNPxJ7ZEBC-L0/edit?usp=sharing


https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 */



package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="GameTeleOPIMUAssist", group="Linear OpMode")
public class DriveWithIMU extends LinearOpMode {
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double  ArmPos = (MAX_POS - MIN_POS) / 2;   //Servo Pos Vars
    double  GripPos = (MAX_POS - MIN_POS) / 2;
    double  Speed = 0.6;
    double retainTime;
    double twr;
    double axial;
    double turnSpeed;
    boolean doTele=true;
    DigitalChannel armLimit;  //Magnetic Limit Switch For Gravity Counter
    DigitalChannel TwrLimit;
    DigitalChannel ArmMag;
    double teleFill= 1;
    double desiredYaw =0;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;  //Drive Motors
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftCH = null;       //Tower Motors
    private DcMotor rightCH = null;
    Servo Arm;                //servos
    Servo Gripper;
    static final double     P_TURN_GAIN            = 0.02;
    NormalizedColorSensor colorSensor;  //color sensors
    View relativeLayout;
    private IMU             imu         = null;      // Control/Expansion Hub IMU
    double headingError=0;


    @Override
    public void runOpMode() throws InterruptedException {
        armLimit = hardwareMap.get(DigitalChannel.class, "magnet");  //Init MagSensor
        Arm = hardwareMap.get(Servo.class, "Arm");              //Init servos
        Gripper = hardwareMap.get(Servo.class, "Gripper");
        armLimit.setMode(DigitalChannel.Mode.INPUT);
        TwrLimit = hardwareMap.get(DigitalChannel.class, "TwrLimit");
        TwrLimit.setMode(DigitalChannel.Mode.INPUT);
        ArmMag = hardwareMap.get(DigitalChannel.class, "ArmMag");
        ArmMag.setMode(DigitalChannel.Mode.INPUT);
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        //Match names to config thingy
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        leftCH = hardwareMap.get(DcMotor.class, "leftCH");
        rightCH = hardwareMap.get(DcMotor.class, "rightCH");
        imu.resetYaw();
        try {
            runSample(); // actually execute the sample
        } finally {
            // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
            // as pure white, but it's too much work to dig out what actually was used, and this is good
            // enough to at least make the screen reasonable again.
            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.BLACK);
                }
            });
        }
    }
    protected void runSample() throws InterruptedException {
        // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
        // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
        // can give very low values (depending on the lighting conditions), which only use a small part
        // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
        // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
        // colors will report at or near 1, and you won't be able to determine what color you are
        // actually looking at. For this reason, it's better to err on the side of a lower gain
        // (but always greater than  or equal to 1).
        float gain = 3;

        final float[] hsvValues = new float[3];

        // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
        // state of the X button on the gamepad
        boolean xButtonPreviouslyPressed = false;
        boolean xButtonCurrentlyPressed = false;

        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "Color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        //SET MOTOR DIRECTIONS
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftCH.setDirection(DcMotor.Direction.REVERSE);
        rightCH.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        //telemetry.addData("Press Start Button To Begin");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            xButtonCurrentlyPressed = gamepad1.x;

            // If the button state is different than what it was, then act
            if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
                // If the button is (now) down, then toggle the light
                if (xButtonCurrentlyPressed) {
                    if (colorSensor instanceof SwitchableLight) {
                        SwitchableLight light = (SwitchableLight)colorSensor;
                        light.enableLight(!light.isLightOn());
                    }
                }
            }
            xButtonPreviouslyPressed = xButtonCurrentlyPressed;
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            //Speed = gamepad2.a ? 0.85 : 0.6;
            if(gamepad2.left_trigger<0.5&&gamepad2.right_trigger<0.5){
                Speed =0.6;  // Normal Speed
            }else if(gamepad2.left_trigger>0.5&&gamepad2.right_trigger<0.5){
                Speed = 0.85;  //Fast Mode
            }else if(gamepad2.left_trigger<0.5&&gamepad2.right_trigger>0.5){
                Speed=0.4;  //Slow Mode
            }
            double max;

            // POV Mode guses left joystick to go forward & strafe, and right joystick to rotate.
            axial   = -gamepad2.left_stick_y*Speed;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad2.left_stick_x*Speed;
            double yaw     =  gamepad2.right_stick_x*Speed;
            twr = gamepad1.left_stick_y*0.58;
            ArmPos = -gamepad1.right_stick_y+0.5;
            if(gamepad2.a){
                lateral=0;
            }
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            if(armLimit.getState()){  //gravity counter for new Arm Ctrl
                twr=twr-0.08;
            }
            if(!armLimit.getState()&&twr<0){
                twr=0;
            }
            if(TwrLimit.getState()&&gamepad1.y){
                twr=0.75;
            }
            if(ArmMag.getState()&&gamepad1.y){
                ArmPos=-1;
            }
            if(gamepad1.left_trigger>0.5){
                twr=-0.3;
            }
            if(gamepad1.right_trigger>0.5){
                twr=0.8;
            }


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }



            if(!gamepad1.dpad_left&&!gamepad1.dpad_right){
                GripPos=0.555;
            }else if(!gamepad1.dpad_left&&gamepad1.dpad_right){
                GripPos=0.95;
            }else if(gamepad1.dpad_left&&!gamepad1.dpad_right){
                GripPos=0.3;
            }

            

            //Send Power To Motors
            if(gamepad2.a){
                desiredYaw=getHeading();
                //lateral=0;
                turnSpeed = getSteeringCorrection(0, 0.2);
                if(turnSpeed>0){
                    //either increase left or rights
                     leftFrontPower=leftFrontPower*1.2;
                     leftBackPower=leftBackPower*1.2;

                }else if(turnSpeed<0){
                    //whichever other ones
                     rightFrontPower=rightFrontPower*1.2;
                     rightBackPower=rightBackPower*1.2;
                }else if(turnSpeed==0){
                    //nothing
                    // leftFrontPower=leftFrontPower;
                    // leftBackPower=leftBackPower;
                    // rightFrontPower=rightFrontPower;
                    // rightBackPower=rightBackPower;
                }
            }
            if(gamepad1.a){
                twr=0.8;
                leftCH.setPower(twr);
                rightCH.setPower(twr);
                Thread.sleep(5000);
                twr=0.0;
                leftCH.setPower(twr);
                rightCH.setPower(twr);
            }
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower*0.76);
            rightBackDrive.setPower(rightBackPower*0.76);
            leftCH.setPower(twr);
            rightCH.setPower(twr);
            Arm.setPosition(ArmPos);
            Gripper.setPosition(GripPos);
            if(doTele) {
                // print out data from drive motors and color sensor
                telemetry.addData("Status", "Run Time: " + runtime.toString());                                 //Runtime
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);         //FrontDrivePwr
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);           //BackDrivePwr
                telemetry.addData("TowerPwr ", twr);                                                            //TowerPwr
                telemetry.addLine()
                        .addData("Red", "%.3f", colors.red)                                                     //ColorSensorStuff
                        .addData("Green", "%.3f", colors.green)
                        .addData("Blue", "%.3f", colors.blue);
                telemetry.addLine()
                        .addData("Hue", "%.3f", hsvValues[0])
                        .addData("Saturation", "%.3f", hsvValues[1])
                        .addData("Value", "%.3f", hsvValues[2]);
                telemetry.addData("Alpha", "%.3f", colors.alpha);
                if (colorSensor instanceof DistanceSensor) {
                    telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
                }
                if (armLimit.getState() == false) {                         //GravityCounterState
                    telemetry.addData("Gravity-Counter", "DISABLED");
                } else {
                    telemetry.addData("Gravity-Counter", "ENABLED");
                }
                telemetry.addData("TwrLimit", TwrLimit.getState());
                telemetry.addData("ArmMag", ArmMag.getState());
                telemetry.addData("Arm Position", "%5.2f", ArmPos);
                telemetry.addData("Gripper Pos", "%5.2f", GripPos);
                telemetry.update();
            }
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
                }
            });
        }
    }
    public void ascend() {
        retainTime = runtime.seconds();
        if(!(retainTime >runtime.seconds()+5)){
            twr = -0.5;
            axial = -0.25;
        }
        while(!(gamepad1.right_trigger >0.5)){
            twr = 0.85;
        }
    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        //targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = desiredHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }
}
