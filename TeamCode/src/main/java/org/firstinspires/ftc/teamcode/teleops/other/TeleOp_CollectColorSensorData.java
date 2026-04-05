package org.firstinspires.ftc.teamcode.teleops.other;

import static org.firstinspires.ftc.teamcode.RobotState.validStartPose;
import static org.firstinspires.ftc.teamcode.Utils.addLine;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.opencv.core.Scalar;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.Locale;

@TeleOp(name = "Collect Color Sensor Data", group = "C")
@Disabled
public class TeleOp_CollectColorSensorData extends OpMode {
    public TeleOpController teleop;
    public Robot robot;
    public TelemetryUtils tm;
    public FileWriter fw;
    public BufferedWriter bw;

    @Override
    public void init() {
        File dir = new File(hardwareMap.appContext.getFilesDir().toURI());
        File file = new File(dir, "colors_0.csv");
        RobotState.color = RobotConstants.Color.BLUE;
        Pose pose = loadOdometryPosition();
        validStartPose = pose != null;
        RobotState.pose = validStartPose ? pose : new Pose();
        RobotState.auto = false;
        robot = new Robot(hardwareMap, telemetry, true);
        robot.drivetrain.follower.setPose(RobotState.pose);
        robot.drivetrain.follower.startTeleopDrive();
        teleop = new TeleOpController(robot, gamepad1, gamepad2);
        tm = robot.drivetrain.tm;
        int i = 0;
        while (true) {
            try {
                if (file.createNewFile()) break;
            } catch (IOException e) {
                tm.print(Arrays.toString(e.getStackTrace()));
                tm.update();
                throw new RuntimeException(e);
            }
            i++;
            file = new File(dir, "colors_" + i + ".csv");
        }
        try {
            fw = new FileWriter(file, true);
            bw = new BufferedWriter(fw);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        if (!validStartPose)
            tm.warn(TelemetryUtils.ErrorLevel.MEDIUM, "Robot Centric driving will be used until the position is reset");
        else tm.print("Field Centric Driving", "✅");
        tm.print("Color", "🟦🟦Blue🟦🟦");
    }

    @Override
    public void start() {
        teleop.start();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            Scalar argbA = robot.colorSensor.getARGB();
            Scalar argbB = robot.colorSensor.getARGBB();
            double distanceA = robot.colorSensor.getInchesA();
            double distanceB = robot.colorSensor.getInchesB();
            addLine(bw, String.format(Locale.US, "Green,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                argbA.val[0],
                argbA.val[1],
                argbA.val[2],
                argbA.val[3],
                distanceA,
                argbB.val[0],
                argbB.val[1],
                argbB.val[2],
                argbB.val[3],
                distanceB));
        }
        if (gamepad1.b) {
            Scalar argbA = robot.colorSensor.getARGB();
            Scalar argbB = robot.colorSensor.getARGBB();
            double distanceA = robot.colorSensor.getInchesA();
            double distanceB = robot.colorSensor.getInchesB();
            addLine(bw, String.format(Locale.US, "Purple,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                argbA.val[0],
                argbA.val[1],
                argbA.val[2],
                argbA.val[3],
                distanceA,
                argbB.val[0],
                argbB.val[1],
                argbB.val[2],
                argbB.val[3],
                distanceB));
        }
        if (gamepad1.x) {
            Scalar argbA = robot.colorSensor.getARGB();
            Scalar argbB = robot.colorSensor.getARGBB();
            double distanceA = robot.colorSensor.getInchesA();
            double distanceB = robot.colorSensor.getInchesB();
            addLine(bw, String.format(Locale.US, "Unknown,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                argbA.val[0],
                argbA.val[1],
                argbA.val[2],
                argbA.val[3],
                distanceA,
                argbB.val[0],
                argbB.val[1],
                argbB.val[2],
                argbB.val[3],
                distanceB));
        }
        teleop.drivetrainLogic(validStartPose);
        teleop.indexerUpdate();
        teleop.updateIntake();
        teleop.feederLogic();
        teleop.updateIndexerTeleOp();
        teleop.updateLauncherTeleOp();
        teleop.update();
    }


    @Override
    public void stop() {
        teleop.stop();
        try {
            bw.close();
            fw.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
