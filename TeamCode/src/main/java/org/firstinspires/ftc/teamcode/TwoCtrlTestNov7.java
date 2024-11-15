
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

@TeleOp(name="2P Test 11/7", group="Linear OpMode")
public class TwoCtrlTestNov7 extends LinearOpMode {
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double  ArmPos = (MAX_POS - MIN_POS) / 2;   //Servo Pos Vars
    double  GripPos = (MAX_POS - MIN_POS) / 2;
    double  Speed = 0.6;
    DigitalChannel armLimit;  //Magnetic Limit Switch For Gravity Counter
    DigitalChannel TwrLimit;
    DigitalChannel ArmMag;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;  //Drive Motors
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftCH = null;       //Tower Motors
    private DcMotor rightCH = null;
    Servo Arm;                //servos
    Servo Gripper;
    NormalizedColorSensor colorSensor;  //color sensors
    View relativeLayout;

    @Override
    public void runOpMode() {
        armLimit = hardwareMap.get(DigitalChannel.class, "magnet");  //Init MagSensor
        Arm = hardwareMap.get(Servo.class, "Arm");              //Init servos
        Gripper = hardwareMap.get(Servo.class, "Gripper");
        armLimit.setMode(DigitalChannel.Mode.INPUT);
        TwrLimit = hardwareMap.get(DigitalChannel.class, "TwrLimit");
        TwrLimit.setMode(DigitalChannel.Mode.INPUT);
        ArmMag = hardwareMap.get(DigitalChannel.class, "ArmMagnet");
        ArmMag.setMode((DigitalChannel.Mode.INPUT));

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        //Match names to config thingy
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        leftCH = hardwareMap.get(DcMotor.class, "leftCH");
        rightCH = hardwareMap.get(DcMotor.class, "rightCH");
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
    protected void runSample() {
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
            Speed = gamepad2.a ? 0.85 : 0.6;
            double max;

            // POV Mode guses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad2.left_stick_y*Speed;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad2.left_stick_x*Speed;
            double yaw     =  gamepad2.right_stick_x*Speed;
            double twr = gamepad1.left_stick_y*0.58;
            ArmPos = -gamepad1.right_stick_y+0.5;
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
            //maybe do if(!gamepad1.left_stick_y<0.1&&!armLimit.getState) to make it so arm
            //cannot be moved up farther than limit when moving joystick.
//            if(gamepad1.y){
//                while(!TwrLimit.getState()) {
//                    twr = 0.75;
//                }
//            }
            if(TwrLimit.getState()&&gamepad1.y){
                twr=0.75;
            }
            if(ArmMag.getState()&&gamepad1.y){
                ArmPos=-0.75;
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


            //Use Dpad to control the tower movement variables.
            // if(!gamepad1.left_stick_y>0.5&&!gamepad1.left_stick_y<0.5){
            //     //Determine if motor GravityCounter is needed, based off magnetic sensor
            //     if (!armLimit.getState()) {
            //         twr=0.0;  //No Buttons Pressed, Arm At Max
            //     } else {
            //         twr=-0.075;  //No Buttons Pressed, PWR to counter weight
            //     }
            // } else if (!gamepad1.dpad_down&&gamepad1.dpad_up) {
            //     twr=-0.5;
            // } else if (gamepad1.dpad_down&&!gamepad1.dpad_up) {
            //     twr=0.5;
            // }


            // if(!gamepad1.y&&!gamepad1.a){
            //     ArmPos=0.5;
            // }else if(!gamepad1.y&&gamepad1.a){  //arm down
            //     ArmPos=1.05;
            // }else if(gamepad1.y&&!gamepad1.a){  //arm up
            //     ArmPos=0.3;
            // }


            if(!gamepad1.dpad_left&&!gamepad1.dpad_right){
                GripPos=0.525;
            }else if(!gamepad1.dpad_left&&gamepad1.dpad_right){
                GripPos=0.95;
            }else if(gamepad1.dpad_left&&!gamepad1.dpad_right){
                GripPos=0.3;
            }

            //Send Power To Motors
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower*0.76);
            rightBackDrive.setPower(rightBackPower*0.76);
            leftCH.setPower(twr);
            rightCH.setPower(twr);
            Arm.setPosition(ArmPos);
            Gripper.setPosition(GripPos);

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
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
                }
            });
        }
    }}
