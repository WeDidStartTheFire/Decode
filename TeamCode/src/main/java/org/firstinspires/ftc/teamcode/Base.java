package org.firstinspires.ftc.teamcode;

import static java.lang.Math.*;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import com.qualcomm.hardware.rev.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.vision.*;
import org.firstinspires.ftc.vision.apriltag.*;

/** Base class that contains common methods and other configuration. */
public abstract class Base extends LinearOpMode {
    private static final double LIFT_VEL = 1500;
    public static final double GOAL_ENCODERS = 2000;
//    private TfodProcessor tfod;
    private static final ElapsedTime runtime = new ElapsedTime();
    // All non-primitive data types initialize to null on default.
    public DcMotorEx lf, lb, rf, rb, carWashMotor, pixelLiftingMotor;
    public Servo droneServo, pixelBackServo, pixelLockingServo, trayTiltingServo;
    public TouchSensor touchSensor;
    public side stageSide;
    public color allianceColor;
    private IMU imu;
    /*
     - Calculate the COUNTS_PER_INCH for your specific drive train.
     - Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
     - For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
     - For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
     - This is gearing DOWN for less speed and more torque.
     - For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    //*/
    public String           hubName;
    public static final String SMALL_WHEEL_ROBOT_NAME = "Expansion Hub 2";
    public static final String LARGE_WHEEL_ROBOT_NAME = "Control Hub";
    public static final double SMALL_WHEEL_DIAMETER = 3.77953;
    public static final double LARGE_WHEEL_DIAMETER = 5.511811;
    static double           WHEEL_DIAMETER_INCHES;
    {
        try {
            hubName = hardwareMap.get(String.class, "Control Hub");
            print("Hub Name", hubName);
            if (SMALL_WHEEL_ROBOT_NAME.equals(hubName)) {
                WHEEL_DIAMETER_INCHES = SMALL_WHEEL_DIAMETER;     // For figuring out circumference
            } else if (LARGE_WHEEL_ROBOT_NAME.equals(hubName)) {
                WHEEL_DIAMETER_INCHES = LARGE_WHEEL_DIAMETER;     // For figuring out circumference
            } else {
                WHEEL_DIAMETER_INCHES = SMALL_WHEEL_DIAMETER;     // Default value
            }
        } catch (Exception e) {
            WHEEL_DIAMETER_INCHES = SMALL_WHEEL_DIAMETER;
            telemetry.addData("Error", e);
            telemetry.update();
        }
    }
    static final double     COUNTS_PER_MOTOR_REV    = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0);
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    static final double     TILE_LENGTH             = 23.25;
    static final double     STRAFE_FRONT_MODIFIER   = 1.3;
    static final double     B                       = 1.1375;
    static final double     M                       = 0.889;
    static final double     TURN_SPEED              = 0.5;
    private static final int WAIT_TIME              = 100;
                 double     velocity                = 2000;
    public double x;
    public VisionPortal visionPortal;
    public String tfodModelName;
    private AprilTagProcessor tagProcessor;

    private static final IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    ));

    /** Color options for the team prop. Options: red, blue, none **/
    public enum color { red, blue, none }

    /** Side of the robot. Options: front, back **/
    public enum side { front, back }

    /** Directions. Options: left, right, forward, backward **/
    public enum dir { left, right, forward, backward }

    /** Initializes all hardware devices on the robot.
     * @param teamColor The color of the team prop.
     * @param useCam Should the camera be initialized? **/
    public void setup(color teamColor, boolean useCam) {
        if (teamColor == color.red) {
            tfodModelName = "Prop_Red.tflite";
        } else if (teamColor == color.blue){
            tfodModelName = "Prop_Blue.tflite";
        } else if (useCam){
            print("Warning","teamColor not specified");
            useCam = false;
        }

        imu = hardwareMap.get(IMU.class, "imu");
        if (!imu.initialize(IMU_PARAMETERS)){
            throw new RuntimeException("IMU initialization failed");
        }
        imu.resetYaw();

        // The following try catch statements "check" if a motor is connected. If it isn't, it sets
        // that motor's value to null. Later, we check if that value is null. If it is, we don't
        // run the motor.
        try {
            lf = hardwareMap.get(DcMotorEx.class, "leftFront");
            lb = hardwareMap.get(DcMotorEx.class, "leftBack");
            rf = hardwareMap.get(DcMotorEx.class, "rightFront");
            rb = hardwareMap.get(DcMotorEx.class, "rightBack");
        } catch (IllegalArgumentException e) {except("At least one drive train motor is not connected, so all will be disabled"); lf = lb = rf = rb = null;}
        // If given an error, the motor is already null
        try {carWashMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");}catch (IllegalArgumentException e){except("carWashMotor (liftMotor) not connected");}
        try {pixelLiftingMotor = hardwareMap.get(DcMotorEx.class,"pixelLiftingMotor");}catch (IllegalArgumentException e){except("pixelLiftingMotor not connected");}
        try {droneServo = hardwareMap.get(Servo.class, "droneServo");}catch (IllegalArgumentException e){except("droneServo not connected");}
        try {pixelBackServo = hardwareMap.get(Servo.class,"pixelBackServo");}catch (IllegalArgumentException e){except("pixelBackServo not connected");}
        try {pixelLockingServo = hardwareMap.get(Servo.class, "pixelFrontServo");}catch (IllegalArgumentException e){except("pixelFrontServo not connected");}
        try {trayTiltingServo = hardwareMap.get(Servo.class,"trayTiltingServo");}catch (IllegalArgumentException e){except("trayTiltingServo not connected");}
        try {touchSensor = hardwareMap.get(TouchSensor.class,"touchSensor");}catch (IllegalArgumentException e){except("touchSensor not connected");}
       if (useCam) {
            try {
                WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
                initProcessors(cam);
            } catch (IllegalArgumentException e) {except("Webcam not connected");}
        }

        if (lf != null) {
            lf.setDirection(DcMotorEx.Direction.REVERSE);
            lb.setDirection(DcMotorEx.Direction.REVERSE);
            rf.setDirection(DcMotorEx.Direction.FORWARD);
            rb.setDirection(DcMotorEx.Direction.FORWARD);

            lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            lb.setTargetPosition(lb.getCurrentPosition());
            rb.setTargetPosition(rb.getCurrentPosition());
            lf.setTargetPosition(lf.getCurrentPosition());
            rf.setTargetPosition(rf.getCurrentPosition());
        }

        if (pixelLiftingMotor != null) {
            pixelLiftingMotor.setDirection(DcMotorEx.Direction.REVERSE);
            pixelLiftingMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            pixelLiftingMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            pixelLiftingMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }


        print("Status", "Initialized");
        print("Hub Name", hubName);
        update();

        allianceColor = teamColor;

        if (droneServo != null) { droneServo.setPosition(1); }
        if (pixelLockingServo != null){ pixelLockingServo.setPosition(0); }
        waitForStart();
        runtime.reset();
    }

    /** Initializes all hardware devices on the robot.
     * Note: When called without useCam manually set, useCam defaults to true.
     * @param teamColor The color of the team prop. **/
    public void setup(color teamColor){
        setup(teamColor, true);
    }

    /** Initializes all hardware devices on the robot. **/
    public void setup() { setup(color.none, false); }

    /**
     * Initializes all hardware devices on the robot.
     * @param isRed Is the team prop red?
     * @deprecated - use setup() with a color instead
     */
    @Deprecated
    public void setup(boolean isRed) { if (isRed) { setup(color.red,true); } else { setup(color.blue,true); } }

    /** Drives using encoder velocity. An inches value of zero will cause the robot to drive until manually stopped.
     * @param inches Amount of inches to drive.
     * @param direction (opt.) Direction to drive if inches is zero.**/
    private void encoderDrive(double inches, dir direction) {
        int lfTarget = 0;
        int rfTarget = 0;

        // Ensure that the OpMode is still active
        if (opModeIsActive() && lf != null) {
            setMotorModes(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            setMotorModes(DcMotorEx.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            if (inches != 0) {
                runtime.reset();
                lb.setVelocity(velocity * signum(inches));
                rb.setVelocity(velocity * signum(inches));
                lf.setVelocity(velocity * signum(inches));
                rf.setVelocity(velocity * signum(inches));
            }
            else if (direction == dir.forward){
                lb.setVelocity(velocity);
                rb.setVelocity(velocity);
                lf.setVelocity(velocity);
                rf.setVelocity(velocity);
            }
            else if (direction == dir.backward){
                lb.setVelocity(-velocity);
                rb.setVelocity(-velocity);
                lf.setVelocity(-velocity);
                rf.setVelocity(-velocity);
            }

            if (inches != 0) {
                inches = signum(inches) * (abs(inches) + B) / M;
            }

            double duration = abs(inches * COUNTS_PER_INCH / velocity);

            while (opModeIsActive() && (runtime.seconds() < duration) && inches != 0) {
                // Display it for the driver.
                print("Angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                print("Running to",  " " + lfTarget + ":" + rfTarget);
                print("Currently at",  " at "+ lf.getCurrentPosition() + ":"+ rf.getCurrentPosition());
                update();
            }
            if (inches != 0) {
              stopRobot();
            }
        }
    }

    /** Turns the robot a specified number of degrees. Positive values turn right,
     * negative values turn left.
     * @param degrees The amount of degrees to turn.
     * @param direction (opt.) Direction to turn if degrees is zero.
     */
    public void turn(double degrees, dir direction) {
        double direct = 0;
        if (direction == dir.left) { direct = -1; }
        else if (direction == dir.right) { direct = 1; }
        sleep(100);
        degrees *= -1;
        degrees -= imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        imu.resetYaw();
        double tolerance = 1;
        double startAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double currentAngle;
        double initialGoalAngle = startAngle + degrees;
        double correctedGoalAngle = initialGoalAngle;
        double difference = 999;
        double turnModifier;
        double turnPower;
        if (abs(initialGoalAngle) > 180) {
            correctedGoalAngle -= abs(initialGoalAngle) / initialGoalAngle * 360;
        }
        while (opModeIsActive() && (difference > tolerance) && degrees != 0) {
            currentAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            difference = min(abs(initialGoalAngle - currentAngle), abs(correctedGoalAngle - currentAngle));
            turnModifier = min(1, (difference + 3) / 30);
            turnPower = degrees / abs(degrees) * TURN_SPEED * turnModifier * direct;
            setMotorPowers(-turnPower, turnPower, -turnPower, turnPower);
            currentAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            difference = min(abs(initialGoalAngle - currentAngle), abs(correctedGoalAngle - currentAngle));
            print("Corrected Goal", correctedGoalAngle);
            print("Initial Goal", initialGoalAngle);
            print("Start", startAngle);
            print("Angle", currentAngle);
            print("Distance from goal", difference);
            update();
        }
        stopRobot();
        imu.resetYaw();
        sleep(WAIT_TIME);
    }


    private void setMotorModes(DcMotor.RunMode mode) {
        if (lb != null) {
            lf.setMode(mode);
            lb.setMode(mode);
            rf.setMode(mode);
            rb.setMode(mode);
        }
    }

    private void setMotorPowers(double lbPower, double rbPower, double lfPower, double rfPower) {
        if (lb != null) {
            lb.setPower(lbPower);
            rb.setPower(rbPower);
            lf.setPower(lfPower);
            rf.setPower(rfPower);
        }
    }


    /** Turns the robot a specified number of degrees. Positive values turn right,
     * negative values turn left. A degrees value of zero will cause the robot to turn until manually stopped.
     * @param degrees The amount of degrees to turn.
     */
    public void turn(double degrees){
        turn(degrees, dir.right);
    }

    /** Strafes left or right for a specified number of inches. An inches value of zero will cause the
     * robot to strafe until manually stopped.
     * @param inches Amount of inches to strafe.
     * @param direction Direction to strafe in.**/
    public void strafe(double inches, dir direction) {

        if (opModeIsActive() && lf != null) {
            setMotorModes(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            setMotorModes(DcMotorEx.RunMode.RUN_USING_ENCODER);

            double d = 0;

            if (direction == dir.right) {
                d = -1;
            } else if (direction == dir.left){
                d = 1;
            }

            runtime.reset();
            lb.setVelocity(velocity * d);
            rb.setVelocity(-velocity * d);
            lf.setVelocity(-velocity * STRAFE_FRONT_MODIFIER * d);
            rf.setVelocity(velocity * STRAFE_FRONT_MODIFIER * d);
            if (inches != 0) {
                inches = (abs(inches) + 1.0125) / 0.7155;
            }

            double duration = abs(inches * COUNTS_PER_INCH / velocity);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < duration) && inches != 0) {
                print("Strafing until",  duration + " seconds");
                print("Currently at",  runtime.seconds() + " seconds");
                update();
            }
            if (inches != 0) {
                stopRobot();
            }
            print("Strafing", "Complete");
            update();

        }
        sleep(WAIT_TIME);
    }

    /** Strafes right for a specified number of inches. An inches value of zero will cause the
     * robot to strafe until manually stopped.
     * @param inches Amount of inches to strafe.
     */
    public void strafe(double inches){
        strafe(inches, dir.right);
    }

    /** Changes the velocity.
     * @param speed New velocity value.
     */
    public void setSpeed(double speed) {
        velocity = speed;
    }

    /** Drives the specified number of inches. Negative values will drive backwards.
     * An inches value of zero will cause the robot to drive until manually stopped.
     * @param inches Amount of inches to drive.
     * @param direction (opt.) Direction to drive if inches is zero.**/
    public void drive(double inches, dir direction) {
        //double startAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int checks = 1; // Number of times the robot will check its orientation during a single drive movement and correct itself
        if (inches != 0) {
            for (int i = 0; i < checks; i++) {
                encoderDrive(inches / checks, direction);
                //turn(startAngle - imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            }
            stopRobot();
        } else {
            encoderDrive(0,direction);
        }
    }

    /** Drives the specified number of inches. Negative values will drive backwards.
     * An inches value of zero will cause the robot to drive until manually stopped.
     * @param inches Amount of inches to drive. **/
    public void drive(double inches){
        drive(inches, dir.forward);
    }

    /** Converts an amount of tiles on the game board to an amount of inches.
     * @param tiles The value of tiles to be converted. **/
    public double tilesToInches(double tiles) {return tiles * TILE_LENGTH;}

    /** Stops all drive train motors on the robot. **/
    public void stopRobot() {
        if (lb != null) {
            setMotorPowers(0, 0, 0, 0);
            lb.setVelocity(0);
            rb.setVelocity(0);
            lf.setVelocity(0);
            rf.setVelocity(0);

            // Set target position to avoid an error
            lb.setTargetPosition(lb.getCurrentPosition());
            rb.setTargetPosition(rb.getCurrentPosition());
            lf.setTargetPosition(lf.getCurrentPosition());
            rf.setTargetPosition(rf.getCurrentPosition());

            // Turn On RUN_TO_POSITION
            setMotorModes(DcMotorEx.RunMode.RUN_TO_POSITION);

            // Stop all motion
            lb.setTargetPosition(lb.getCurrentPosition());
            rb.setTargetPosition(rb.getCurrentPosition());
            lf.setTargetPosition(lf.getCurrentPosition());
            rf.setTargetPosition(rf.getCurrentPosition());

            // Turn off RUN_TO_POSITION
            setMotorModes(DcMotorEx.RunMode.RUN_USING_ENCODER);

            sleep(WAIT_TIME);
        }
    }
    /** Returns information about a tag with the specified ID if it is currently detected.
     * @param id ID of tag to detect.
     * @return Information about the tag detected. **/
    @Nullable
    public AprilTagDetection tagDetections(int id) {
        int i;
        for (i = 0; i < tagProcessor.getDetections().size(); i++)
        {
            if (tagProcessor.getDetections().get(i).id == id){
                return tagProcessor.getDetections().get(i);
            }
        }
        return null;
    }

    /** Returns information about a tag with the specified ID if it is detected within a designated timeout period.
     * @param id ID of tag to detect.
     * @param timeout Detection timeout (seconds).
     * @return Information about the tag detected. **/
    @Nullable
    public AprilTagDetection tagDetections(int id, double timeout) {
        AprilTagDetection a;
        double t = runtime.milliseconds() + timeout;
        while (opModeIsActive() && (runtime.milliseconds() < t)) {
            a = tagDetections(id);
            if (a != null) {
                return a;
            }
        }
        return null;
    }

    /** Aligns the robot in front of the AprilTag.
     * @param id ID of tag to align with.
     */
    public void align(int id) {
        AprilTagDetection a = tagDetections(id, 1);
        turn(0);
            while (opModeIsActive() && (a != null && (abs(a.ftcPose.x) > 0.5 || abs(a.ftcPose.yaw) > 0.5))) {
                a = tagDetections(id, 1);
                if (a == null) { return; }
                print("Strafe", a.ftcPose.x);
                strafe(a.ftcPose.x);
                a = tagDetections(id, 1);
                if (a == null) { return; }
                print("Drive", -a.ftcPose.y + 5);
                drive(-a.ftcPose.y + 2);
                a = tagDetections(id, 1);
                if (a == null) { return; }
                print("Turn", a.ftcPose.yaw / 2);
                turn(a.ftcPose.yaw / 2);
                update();
        }
    }

    /** Sends an exception message to Driver Station telemetry.
     * @param e The exception. **/
    public final void except(Object e) {
        print("Exception", e);
    }

    /** Sleep a specified number of seconds.
     * @param seconds The amount of seconds to sleep. **/
    public final void s (double seconds){ sleep((long) seconds * 1000);}

    /** Moves the lift motor a specified number of inches.
     * @param encoders Number of encoders to turn the motor.
     */
    public void moveLift(double encoders) {
        if (pixelLiftingMotor != null) {
            pixelLiftingMotor.setVelocity(LIFT_VEL * signum(encoders));

            runtime.reset();
            while (opModeIsActive() && pixelLiftingMotor.getCurrentPosition() < encoders) {
                // Display it for the driver.
                print("Angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                print("Currently at", " at " + pixelLiftingMotor.getCurrentPosition());
                print("Goal", encoders);
                update();
            }
            pixelLiftingMotor.setVelocity(0);
        }
    }

    public void retractLift() {
        if (pixelLiftingMotor != null) {
            pixelLiftingMotor.setVelocity(-LIFT_VEL);
            while (opModeIsActive() && pixelLiftingMotor.getCurrentPosition() > 0) {
                // Display it for the driver.
                print("Angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                print("Currently at", " at " + pixelLiftingMotor.getCurrentPosition());
                print("Goal", 0);
                update();
            }
            pixelLiftingMotor.setVelocity(0);
        }
    }

    private void initProcessors(WebcamName camera){

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(camera);
        builder.enableLiveView(true);
        builder.addProcessor(tagProcessor);

        visionPortal = builder.build();
    }
    /** A less space consuming way to add telemetry.
    * Format: "(caption): (content)" 
    * @param quick If true, instantly update the telemetry. **/
   public void print(String caption, Object content, boolean quick){
       telemetry.addData(caption, content);
       if (quick) {
           update();
       }
   }

   /** A less space consuming way to add telemetry. **/
   public void print(String caption, Object content) { print(caption, content, false); }

   /** A less space consuming way to update the displayed telemetry. **/
  public void update() {telemetry.update();}
}
