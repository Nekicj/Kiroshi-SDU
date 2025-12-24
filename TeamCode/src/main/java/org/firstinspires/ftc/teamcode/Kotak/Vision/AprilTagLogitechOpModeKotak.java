package org.firstinspires.ftc.teamcode.Kotak.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Vision.LogitechAprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
@TeleOp(name = "AprilTag Logitech ID",group = "comptetition")
public class AprilTagLogitechOpModeKotak extends LinearOpMode {

    public LogitechAprilTag vision;
    private FtcDashboard ftcDashboard;

    @Override
    public void runOpMode(){
        vision = new LogitechAprilTag();
        vision.init(hardwareMap,telemetry);
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry,ftcDashboard.getTelemetry());
        ftcDashboard.startCameraStream(vision.getVisionPortal(),30);


        waitForStart();
        while(opModeIsActive()){
            vision.update();
            AprilTagDetection id33 = vision.getDetectionBySpecificId(33);
            if(id33 != null){
                telemetry.addData("id33 String:",id33.toString());
                telemetry.update();
            }
            vision.showTelemetry(id33);



        }
    }
}
