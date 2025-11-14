package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.BaseController;
import org.firstinspires.ftc.teamcode.Utils.asmGamepadEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Config
@TeleOp(name = "Base Op",group = "Competition")
public class baseOp extends LinearOpMode {
    private BaseController baseController;
    private asmGamepadEx driver1;

    private Follower follower;
    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        follower.startTeleopDrive(true);
        follower.setStartingPose(new Pose(0,0,0));

        waitForStart();

        while (opModeIsActive()){

            double forward = -gamepad1.left_stick_y * Math.abs(-gamepad1.left_stick_y);
            double strafe = -gamepad1.left_stick_x * Math.abs(-gamepad1.left_stick_x);
            double rotate = -gamepad1.right_stick_x * Math.abs(-gamepad1.right_stick_x);

            if(forward <= 0.05 && forward >= -0.05){
                forward = 0;
            }
            if(strafe <= 0.05 && strafe >= -0.05){
                strafe = 0;
            }
            if(rotate <= 0.05 && rotate>= -0.05){
                rotate = 0;
            }

            follower.update();
            follower.setTeleOpDrive(
                    forward ,
                    strafe ,
                    rotate *0.7,
                    true // Robot Centric
            );

            telemetry.addData("robot X: ",follower.getPose().getX());
            telemetry.addData("robot Y: ",follower.getPose().getY());
            telemetry.addData("robot heading: ",follower.getPose().getHeading());
            telemetry.addData("robot heading: ",follower.getPose().distanceFrom(new Pose(0,0,0)));



        }
    }

}
