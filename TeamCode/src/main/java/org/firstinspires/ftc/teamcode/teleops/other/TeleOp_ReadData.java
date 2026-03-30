package org.firstinspires.ftc.teamcode.teleops.other;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TelemetryUtils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

@TeleOp(name = "Read Data", group = "C")
@Configurable
public class TeleOp_ReadData extends OpMode {
    public TelemetryUtils tm;
    public static int i = 0;
    public File file = new File("colors_0.csv");

    @Override
    public void init() {
        tm = new TelemetryUtils(telemetry);
        tm.print(file.getName());
        tm.print(file.exists() ? "Exists" : "Does not exist");
        tm.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadUpWasPressed()) {
            i++;
            file = new File("colors_" + i + ".csv");
            tm.print(file.getName());
            tm.print(file.exists() ? "Exists" : "Does not exist");
            tm.update();
        }
        if (gamepad1.dpadDownWasPressed()) {
            i--;
            file = new File("colors_" + i + ".csv");
            tm.print(file.getName());
            tm.print(file.exists() ? "Exists" : "Does not exist");
            tm.update();
        }
    }

    @Override
    public void start() {
        file = new File("colors_" + i + ".csv");
        try {
            FileReader fr = new FileReader(file);
            BufferedReader br = new BufferedReader(fr);
            String[] lines = (String[]) br.lines().toArray();
            for (String line : lines) tm.print(line);
            br.close();
            fr.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        tm.update();
    }

    @Override
    public void loop() {
    }
}
