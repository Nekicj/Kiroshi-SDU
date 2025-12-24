package org.firstinspires.ftc.teamcode.Kotak.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="AprilTag HuskyLens")
public class HuskyAprilTagKotak extends LinearOpMode {

    private HuskyLens huskyLens = null;
    private double targetTagId = -1;

    private double x;
    private double y;
    private double left;
    private double height;
    private double width;
    private double top;
    @Override
    public void runOpMode(){
        huskyLens = hardwareMap.get(HuskyLens.class,"husky");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addData("Init","aa");
        telemetry.update();


        waitForStart();
        boolean tagDetected = false;
        while(opModeIsActive()){

            HuskyLens.Block[] blocks = huskyLens.blocks();

            if (blocks.length > 0) {
                targetTagId = blocks[0].id;
                tagDetected = true;
                telemetry.addData("Tag Id", targetTagId);
                x = blocks[0].x;
                y = blocks[0].y;
                left = blocks[0].left;
                height = blocks[0].height;
                width = blocks[0].width;
                top = blocks[0].top;

            }
            telemetry.addData("Tag 0 x",x);
            telemetry.addData("Tag 0 y",y);

            telemetry.addData("Tag 0 left",left);
            telemetry.addData("Tag 0 height",height);

            telemetry.addData("Tag 0 width",width);

            telemetry.addData("Tag 0 top",top);

            telemetry.addData("isDetected",tagDetected);
            telemetry.update();

        }
    }
}