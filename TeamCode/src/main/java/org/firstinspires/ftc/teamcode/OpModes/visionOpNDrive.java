package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;
import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Controllers.BaseController;
import org.firstinspires.ftc.teamcode.Controllers.Niggantroller;
import org.firstinspires.ftc.teamcode.Utils.asmGamepadEx;
import org.firstinspires.ftc.teamcode.Utils.asmPIDFController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import java.util.List;

@Disabled
@TeleOp(name = "Artifact blobber N Drive", group = "Kotak")
@Config
public class visionOpNDrive extends LinearOpMode {

    private FtcDashboard ftcDashboard;
    private BaseController baseController;
    private asmPIDFController headingPIDFController;
    private asmPIDFController drivePIDFController;
    private asmGamepadEx driver1;
    private Niggantroller niggantroller;

    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.005;
    public static double kF = 0.001;

    public static double kPdrive = 0;
    public static double kIdrive = 0;
    public static double kDdrive = 0;
    public static double kFdrive = 0.1;

    private boolean isCentering = false;
    private double centerX = 160;
    public static double headingOffset = 20;

    private double driveCenter = 65;
    public static double driveOffset = 2;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        niggantroller = new Niggantroller(hardwareMap,telemetry);


        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());
        baseController = new BaseController();
        baseController.initialize(hardwareMap,false);
        driver1 = new asmGamepadEx(gamepad1);

        headingPIDFController = new asmPIDFController(kP, kI, kD,kF);
        headingPIDFController.setSetPoint(centerX);

        drivePIDFController = new asmPIDFController(kPdrive, kIdrive, kDdrive,kFdrive);
        drivePIDFController.setSetPoint(driveCenter);

        ColorBlobLocatorProcessor purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,
                        new Scalar(32, 135, 135),
                        new Scalar(255, 155, 169)))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 0))
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(purpleLocator)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(320, 240))
                .setAutoStopLiveView(true)
                .enableLiveView(true)
                .build();

        ftcDashboard.startCameraStream(visionPortal, 30);

        waitForStart();

        while (opModeIsActive()) {
            headingPIDFController.setPIDF(kP, kI, kD,kF);
            drivePIDFController.setPIDF(kPdrive, kIdrive, kDdrive,kFdrive);

            List<ColorBlobLocatorProcessor.Blob> blobs = purpleLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    100, 20000, blobs);

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    0.5, 1, blobs);

            ColorBlobLocatorProcessor.Util.sortByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs
            );

            double rotatePower = 0;
            double drivePower = 0;


            if (driver1.isRightBumperPressed()) {
                isCentering = !isCentering;
            }


            if (!blobs.isEmpty()) {
                ColorBlobLocatorProcessor.Blob targetBlob = blobs.get(0);

                telemetry.addData("Circularity", targetBlob.getCircularity());
                telemetry.addData("Radius", targetBlob.getCircle().getRadius());
                telemetry.addData("Center", "(" + targetBlob.getCircle().getX() + ", " + targetBlob.getCircle().getY() + ")");
                telemetry.addData("Error", centerX - targetBlob.getCircle().getX());


                if (isCentering) {
                    rotatePower = headingPIDFController.calculate(targetBlob.getCircle().getX());

                    rotatePower = Math.max(-0.3, Math.min(0.3, rotatePower));

                    drivePower = drivePIDFController.calculate(targetBlob.getCircle().getRadius());

                    drivePower = Math.max(-0.6, Math.min(0.6, drivePower));

                    telemetry.addData("rotate error", rotatePower);
                    telemetry.addData("drive error", drivePower);
                }
            } else {
                telemetry.addData("s", "NO NIGGA");
            }

            if (isCentering && !blobs.isEmpty() ) {
                if((blobs.get(0).getCircle().getX() < centerX- headingOffset || blobs.get(0).getCircle().getX() > centerX + headingOffset) && (blobs.get(0).getCircle().getRadius() < driveCenter- driveOffset || blobs.get(0).getCircle().getRadius() > driveCenter + driveOffset)){
                    baseController.update(
                            gamepad1.left_stick_x,
                            drivePower,
                            -rotatePower,
                            1,
                            gamepad1.left_trigger > 0,
                            true
                    );
                }else if(blobs.get(0).getCircle().getX() < centerX- headingOffset || blobs.get(0).getCircle().getX() > centerX + headingOffset){
                    baseController.update(
                            gamepad1.left_stick_x,
                            0,
                            -rotatePower,
                            1,
                            gamepad1.left_trigger > 0,
                            true
                    );
                }else if(blobs.get(0).getCircle().getRadius() < driveCenter- driveOffset || blobs.get(0).getCircle().getRadius() > driveCenter + driveOffset){
                    baseController.update(
                            gamepad1.left_stick_x,
                            drivePower,
                            0,
                            1,
                            gamepad1.left_trigger > 0,
                            true
                    );
                }


            } else {
                baseController.update(
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        1,
                        gamepad1.left_trigger > 0,
                        true
                );
            }

            telemetry.addData("Centering Mode", isCentering);
            telemetry.update();
            niggantroller.update(false);
            niggantroller.intakeEpt(1);
            driver1.update();
        }
    }
}