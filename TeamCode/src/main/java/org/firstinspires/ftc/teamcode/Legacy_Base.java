package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static java.lang.Math.*;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.vision.*;
import org.firstinspires.ftc.vision.apriltag.*;


import static java.util.Locale.US;

import android.os.Environment;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import com.pedropathing.geometry.Pose;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import pedroPathing.constants.Constants;

/** Base class that contains common methods and other configuration. */
@Deprecated
public abstract class Legacy_Base extends LinearOpMode {
    // All non-primitive data types initialize to null on default.
    public DcMotorEx lf, lb, rf, rb, liftMotor, wristMotor, verticalMotorA, verticalMotorB, sorterMotor;
    public Servo wristServoX, wristServoY, basketServo, specimenServo, intakeServo;
    public TouchSensor verticalTouchSensor, horizontalTouchSensor;
    public IMU imu;
    /*
     - Calculate the COUNTS_PER_INCH for your specific drive train.
     - Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
     - For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
     - For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
     - This is gearing DOWN for less speed and more torque.
     - For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    */
    public String hubName;

    public boolean useOdometry = true, useCam = true;
    public VisionPortal visionPortal;
    public Follower follower;
    public Pose currentPose = new Pose();
    public int updates = 0;
    public Double startRuntime;

    public volatile boolean loop = false;
    public volatile boolean running = true;
    public volatile boolean hold = false;
    public volatile boolean holdWrist = false;
    public boolean following = false;

    public boolean auto = false;


    boolean vertRunToPos;

    int liftGoal = 0;
    boolean liftRunToPos;

    static final int[] V_LIFT_BOUNDS = {0, 1950};
    static final int[] V_LIFT_GOALS = {0, 280, 500, 1350, 1950};
    static final double WRIST_MOTOR_POWER = 0.1;

    public void buildPaths() {
    }

    /** Initializes all hardware devices on the robot. */
    public void setup() {
        imu = hardwareMap.get(IMU.class, "imu");
        if (!imu.initialize(IMU_PARAMS)) throw new RuntimeException("IMU initialization failed");
        imu.resetYaw();

        // The following try catch statements "check" if a motor is connected. If it isn't, it sets
        // that motor's value to null. Later, we check if that value is null. If it is, we don't
        // run the motor.
        // Drive train
        try {
            lf = hardwareMap.get(DcMotorEx.class, "leftFront"); // Port 1
            lb = hardwareMap.get(DcMotorEx.class, "leftBack"); // Port 3
            rf = hardwareMap.get(DcMotorEx.class, "rightFront"); // Port 0
            rb = hardwareMap.get(DcMotorEx.class, "rightBack"); // Port 4
        } catch (IllegalArgumentException e) {
            except("At least one drive train motor is not connected, so all will be disabled");
            lf = lb = rf = rb = null;
        }

        // Motors
        try {
            liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor"); // Expansion Hub 0
        } catch (IllegalArgumentException e) {
            except("liftMotor (previously used as carWashMotor) not connected");
        }
        try {
            verticalMotorA = hardwareMap.get(DcMotorEx.class, "verticalMotorA"); // Expansion Hub 2
            verticalMotorB = hardwareMap.get(DcMotorEx.class, "verticalMotorB"); // Expansion Hub 3
        } catch (IllegalArgumentException e) {
            verticalMotorA = verticalMotorB = null;
            except(">= 1 verticalMotor connected; All vertical lift motors disabled");
        }
        try {
            wristMotor = hardwareMap.get(DcMotorEx.class, "wristMotor"); // Expansion Hub 1
        } catch (IllegalArgumentException e) {
            except("wristMotor not connected");
        }
        try {
            sorterMotor = hardwareMap.get(DcMotorEx.class, "sorterMotor"); // Not configured
        } catch (IllegalArgumentException e) {
            except("wristMotor not connected");
        }

        // Servos
        try {
            wristServoX = hardwareMap.get(Servo.class, "wristServoX"); // Expansion Hub 0
        } catch (IllegalArgumentException e) {
            except("wristServoX not connected");
        }
        try {
            wristServoY = hardwareMap.get(Servo.class, "wristServoY"); // Expansion Hub 2
        } catch (IllegalArgumentException e) {
            except("wristServoY not connected");
        }
        try {
            intakeServo = hardwareMap.get(Servo.class, "intakeServo"); // Expansion Hub 1
        } catch (IllegalArgumentException e) {
            except("intakeServo not connected");
        }
        try {
            specimenServo = hardwareMap.get(Servo.class, "specimenServo"); // Control Hub 2
        } catch (IllegalArgumentException e) {
            except("specimenServo not connected");
        }
        try {
            basketServo = hardwareMap.get(Servo.class, "basketServo"); // Control Hub 3
        } catch (IllegalArgumentException e) {
            except("basketServo not connected");
        }

        // Touch Sensors
        try {
            verticalTouchSensor = hardwareMap.get(TouchSensor.class, "verticalTouchSensor"); // Control Hub 0/1
        } catch (IllegalArgumentException e) {
            except("verticalTouchSensor not connected");
        }
        try {
            horizontalTouchSensor = hardwareMap.get(TouchSensor.class, "horizontalTouchSensor"); // Control Hub 2/3
        } catch (IllegalArgumentException e) {
            except("horizontalTouchSensor not connected");
        }

        if (useCam) {
            try {
                WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
                initProcessors(cam);
            } catch (IllegalArgumentException e) {
                except("Webcam not connected");
            }
        }


        if (useOdometry) {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(currentPose);
            follower.startTeleopDrive();
            buildPaths();
        }

        if (lf != null) {
            lf.setDirection(REVERSE);
            lb.setDirection(REVERSE);
            rf.setDirection(DcMotorEx.Direction.FORWARD);
            rb.setDirection(DcMotorEx.Direction.FORWARD);

            if (auto) {
                lf.setZeroPowerBehavior(BRAKE);
                lb.setZeroPowerBehavior(BRAKE);
                rf.setZeroPowerBehavior(BRAKE);
                rb.setZeroPowerBehavior(BRAKE);
            } else {
                lf.setZeroPowerBehavior(FLOAT);
                lb.setZeroPowerBehavior(FLOAT);
                rf.setZeroPowerBehavior(FLOAT);
                rb.setZeroPowerBehavior(FLOAT);
            }

            lb.setTargetPosition(lb.getCurrentPosition());
            rb.setTargetPosition(rb.getCurrentPosition());
            lf.setTargetPosition(lf.getCurrentPosition());
            rf.setTargetPosition(rf.getCurrentPosition());
        }

        if (liftMotor != null) {
            liftMotor.setMode(STOP_AND_RESET_ENCODER);
            liftMotor.setMode(RUN_WITHOUT_ENCODER);
            liftMotor.setZeroPowerBehavior(BRAKE);
        }

        if (verticalMotorA != null) {
            verticalMotorB.setDirection(REVERSE);

            verticalMotorA.setZeroPowerBehavior(BRAKE);
            verticalMotorB.setZeroPowerBehavior(BRAKE);

            verticalMotorA.setMode(STOP_AND_RESET_ENCODER);
            verticalMotorB.setMode(STOP_AND_RESET_ENCODER);

            verticalMotorA.setTargetPosition(verticalMotorA.getCurrentPosition());
            verticalMotorB.setTargetPosition(verticalMotorB.getCurrentPosition());

            verticalMotorA.setMode(RUN_USING_ENCODER);
            verticalMotorB.setMode(RUN_USING_ENCODER);
        }

        if (wristMotor != null) {
            wristMotor.setDirection(REVERSE);
            wristMotor.setZeroPowerBehavior(BRAKE);
            wristMotor.setMode(STOP_AND_RESET_ENCODER);
            wristMotor.setMode(RUN_USING_ENCODER);
        }

        print("Status", "Initialized");
        print("Hub Name", hubName);
        update();

        while (!isStarted() && !isStopRequested()) {
            useOdometry = ((useOdometry || gamepad1.b) && !gamepad1.a);
            print("useOdometry", useOdometry);
            print("Status", "Initialized");
            print("Hub Name", hubName);
            print(":::", ":::");
            print(":::", ":::");
            telemetryAll();
        }
        runtime.reset();
    }

    /**
     * Initializes all hardware devices on the robot.
     *
     * @param startPose Starting position of the robot
     */
    public void setup(Pose startPose) {
        currentPose = startPose;
        setup();
    }

    /**
     * Moves the vertical lift motor a to a specified encoder mark
     *
     * @param encoders Number of encoders from zero position to turn the motor to
     */
    public void moveVerticalLift(double encoders) {
        if (verticalMotorA == null) return;
        if (verticalTouchSensor != null && verticalTouchSensor.isPressed()) {
            verticalMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        verticalMotorA.setMode(RUN_WITHOUT_ENCODER);
        verticalMotorB.setMode(RUN_WITHOUT_ENCODER);
        double basePower;

        while (active()) {
            int vertA = verticalMotorA.getCurrentPosition();
            int vertB = verticalMotorB.getCurrentPosition();

            // Copy the non-zero value if one is zero but the other is large
            if (vertA == 0 && vertB > 100) vertA = vertB;
            if (vertB == 0 && vertA > 100) vertB = vertA;

            int vertAvg = (vertA + vertB) / 2;
            double error = encoders - vertAvg;

            // Stop if close enough
            if (Math.abs(error) < 20) break;

            if (error > 0) basePower = .85;
            else basePower = -.5;

            // Minor correction if one motor is off-center by >10
            if (verticalMotorA.getCurrentPosition() - vertAvg > 10)
                verticalMotorA.setPower(basePower * 0.95);
            else verticalMotorA.setPower(basePower);
            if (verticalMotorB.getCurrentPosition() - vertAvg > 10)
                verticalMotorB.setPower(basePower * 0.95);
            else verticalMotorB.setPower(basePower);

            // If we press the touch sensor, reset encoders
            if (verticalTouchSensor != null && verticalTouchSensor.isPressed()) {
                verticalMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                verticalMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            print("Position", vertAvg);
            print("Goal", encoders);
            print("Error", error);
            print("A Power", verticalMotorA.getPower());
            print("B Power", verticalMotorB.getPower());
            if (!loop) update();
        }

        // Turn off power at the end
        verticalMotorA.setPower(0);
        verticalMotorB.setPower(0);
    }

    /**
     * Holds the vertical lift motor at a specified encoder mark
     *
     * @param vertGoal Number of encoders from zero position to turn the motor to
     */
    public void holdVerticalLift(int vertGoal) {
        if (verticalMotorA == null) return;
        hold = true;
        while (active() && hold) {
            int vertA = verticalMotorA.getCurrentPosition();
            int vertB = verticalMotorB.getCurrentPosition();
            if (vertA == 0 && vertB > 100) vertA = vertB;
            if (vertB == 0 && vertA > 100) vertB = vertA;
            int vertAvg = (vertA + vertB) / 2;
            double power = vertAvg < vertGoal - 20 ? 0.15 : (vertGoal - vertAvg) / 20.0 * .1;
            verticalMotorA.setPower(power);
            verticalMotorB.setPower(power);
            print("Holding Lift at", vertGoal);
            if (!loop) update();
        }
        verticalMotorA.setPower(0);
        verticalMotorB.setPower(0);
    }

    /** Retracts the horizontal lift motor. */
    public void retractVerticalLift() {
        moveVerticalLift(V_LIFT_BOUNDS[0]);
        sleep(10);
        if (verticalTouchSensor != null && verticalTouchSensor.isPressed()) {
            verticalMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** Holds the wrist */
    public void holdWrist(int holdPos) {
        if (wristMotor == null) return;
        holdWrist = true;
        while (holdWrist && active()) {
            int wristMotorPos = wristMotor.getCurrentPosition();
            int error = holdPos - wristMotorPos;
            wristMotor.setPower((abs(error) > 3 ? WRIST_MOTOR_POWER * error / 10.0 : 0) + (wristMotorPos > 30 ? 0.02 : 0));
        }
    }

    public void holdWristOutOfWay() {
        holdWrist(30);
    }

    /**
     * Moves the specimen servo to the specified position.
     *
     * @param position Position between 0 and 1 to move the servo to
     */
    public void moveSpecimenServo(double position) {
        if (specimenServo != null) specimenServo.setPosition(position);
    }

    public void moveWristServoY(double position) {
        if (wristServoY != null) wristServoY.setPosition(position);
    }

    /** Opens the specimen servo. */
    public void openSpecimenServo() {
        moveSpecimenServo(1);
    }

    /** Closes the specimen servo. */
    public void closeSpecimenServo() {
        moveSpecimenServo(0);
    }

    /**
     * Sends an exception message to Driver Station telemetry.
     *
     * @param e The exception.
     */
    public final void except(Object e) {
        print("Exception", e);
    }

    /**
     * Sleep a specified number of seconds.
     *
     * @param seconds The amount of seconds to sleep.
     */
    public final void s(double seconds) {
        sleep((long) seconds * 1000);
    }

    /**
     * Initializes the AprilTag processor.
     *
     * @param camera The camera to use.
     */
    private void initProcessors(WebcamName camera) {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
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

    /**
     * A less space consuming way to add telemetry.
     *
     * @param caption String
     * @param content Object
     */
    public void print(String caption, Object content) {
        telemetry.addData(caption, content);
    }

    /**
     * A less space consuming way to add telemetry.
     *
     * @param content Content to display in telemetry
     */
    public void print(String content) {
        telemetry.addLine(content);
    }

    /** A less space consuming way to update the displayed telemetry. */
    public void update() {
        telemetry.update();
    }

    /**
     * Saves the current pose to a file.
     *
     * @param pos Pose to save
     */
    public void saveOdometryPosition(@NonNull Pose pos) {
        File file = new File(Environment.getExternalStorageDirectory(), "odometryPosition.txt");
        try (FileWriter writer = new FileWriter(file, false)) {
            writer.write(pos.getX() + "," + pos.getY() + "," + pos.getHeading()); // Write the latest position
        } catch (IOException e) {
            print("Error", "Failed to save odometry position: " + e.getMessage());
        }
    }

    /** Adds information messages to telemetry and updates it */
    public void telemetryAll() {
        if (lf == null) telemetry.addData("Drive Train", "Disconnected");
        if (follower != null) print("Following", follower.isBusy());
        if (verticalMotorA == null) print("Vertical Lift Motors", "Disconnected");
        else {
            print("Vertical Motor A Position", verticalMotorA.getCurrentPosition());
            print("Vertical Motor B Position", verticalMotorB.getCurrentPosition());
            print("Vertical Motor Power", (verticalMotorA.getPower() + verticalMotorB.getPower()) / 2.0);
            print("Vertical Run To Position", vertRunToPos);
        }
        if (liftMotor == null) print("Horizontal Lift Motor", "Disconnected");
        else {
            print("Horizontal Lift Motor Position", liftMotor.getCurrentPosition());
            print("Horizontal Lift Motor Power", liftMotor.getPower());
            print("Lift Run To Position", liftRunToPos);
            print("Lift Goal", liftGoal);
        }
        if (wristMotor == null) print("Wrist Motor", "Disconnected");
        else print("Wrist Motor Position", wristMotor.getCurrentPosition());

        if (wristServoX == null) print("Wrist Servo", "Disconnected");
        else print("Wrist Servo Position", wristServoX.getPosition());
        if (intakeServo == null) print("Intake Servo", "Disconnected");
        else print("Intake Servo Position", intakeServo.getPosition());
        if (specimenServo == null) print("Specimen Servo", "Disconnected");
        else print("Specimen Servo Position", specimenServo.getPosition());
        if (basketServo == null) print("Basket Servo", "Disconnected");
        else print("Basket Servo Position", basketServo.getPosition());

        if (verticalTouchSensor == null) print("Touch Sensor", "Disconnected");
        else print("Touch Sensor Pressed", verticalTouchSensor.isPressed());
        if (wristServoX == null) print("Intake Servo", "Disconnected");
        if (useOdometry) {
            if (startRuntime == null)
                startRuntime = runtime.seconds();
            follower.update();
            Pose pos = follower.getPose();
            print(String.format(US, "SparkFun Position :  X: %.2f, Y: %.2f, θ: %.2f°", pos.getX(), pos.getY(), toDegrees(pos.getHeading())));
            updates++;
            print("Updates", updates);
            print("Updates per Second", updates / runtime.seconds());
        } else print("Odometry disabled");

        update();
    }

    /** Returns whether the robot is active. */
    public boolean active() {
        return opModeIsActive() && !isStopRequested() && running;
    }
}