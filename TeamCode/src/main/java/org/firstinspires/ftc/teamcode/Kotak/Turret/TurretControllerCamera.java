package org.firstinspires.ftc.teamcode.Kotak.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Vision.LogitechAprilTag;
import org.firstinspires.ftc.teamcode.Utils.asmPIDController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class TurretControllerCamera {

    private CRServo leftTurretServo = null;
    private CRServo rightTurretServo = null;

    private double servoPower = 0;

    public static double kP = 0.001;
    public static double kI = 0;
    public static double kD = 0.001;
    public static double kF = 0.001;

    public static double tolerance = 10;
    private double center = 320;
    private boolean isCentering = false;

    public LogitechAprilTag vision;
    private FtcDashboard ftcDashboard;

    public Telemetry telemetry;



    private asmPIDController turretPIDController = new asmPIDController(kP,kI,kD);

    public void initialize(HardwareMap hardwareMap, String turretLeftName, String turretRightName, Telemetry telemetry){
        leftTurretServo = hardwareMap.get(CRServo.class,turretLeftName);
        rightTurretServo = hardwareMap.get(CRServo.class,turretRightName);

        turretPIDController.setTarget(center);
        turretPIDController.setTolerance(tolerance);

        vision = new LogitechAprilTag();
        vision.init(hardwareMap,telemetry);
        ftcDashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry,ftcDashboard.getTelemetry());
        ftcDashboard.startCameraStream(vision.getVisionPortal(),30);
//        leftTurretServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update(){
        turretPIDController.setCoefficients(kP,kI,kD);


        if(!isCentering){
            leftTurretServo.setPower(servoPower);
            rightTurretServo.setPower(servoPower);
        }else{
            vision.update();

            AprilTagDetection id33 = vision.getDetectionBySpecificId(33);
            double servopower = 0;
            if(id33!=null){
                vision.showTelemetry(id33);

//                double error = id33.center.x - center;


                servopower = turretPIDController.calculate(id33.center.x);

                servopower = Math.max(-1, Math.min(1, servopower));

            }
            leftTurretServo.setPower(servopower);
            rightTurretServo.setPower(servopower);

        }

    }

    public void setTurretPower(double servoPower){
        this.servoPower = servoPower;
    }

    public void setCentering(){
        isCentering = !isCentering;
    }

    public void showTelemetry(Telemetry telemetry){
        telemetry.addLine("============= Turret ============");
        telemetry.addData("isCentering",isCentering);
        AprilTagDetection id33 = vision.getDetectionBySpecificId(33);
        if(id33!=null){
            telemetry.addData("center april tag",id33.center.x);
            telemetry.addData("error",id33.center.x - center);
        }
        telemetry.update();
    }

}
