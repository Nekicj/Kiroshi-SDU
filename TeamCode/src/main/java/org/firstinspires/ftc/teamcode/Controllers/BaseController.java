    package org.firstinspires.ftc.teamcode.Controllers;

    import com.acmerobotics.dashboard.config.Config;

    import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.qualcomm.robotcore.hardware.PIDCoefficients;

    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
    import org.firstinspires.ftc.teamcode.Utils.asmPIDController;
    import org.firstinspires.ftc.teamcode.Utils.asmPIDFController;

    @Config
    public class BaseController {
        private double targetHeading = 0;
        private asmPIDFController headingPIDF;

        public static double Kp = 0.3;
        public static double Ki = 0;
        public static double Kd = 0.01;
        public static double Kf = 0.01;



        DcMotor Lfront = null;
        DcMotor Rfront = null;
        DcMotor Rback = null;
        DcMotor Lback = null;

        GoBildaPinpointDriver pinpoint = null;



        public void initialize(HardwareMap hardwareMap,boolean useBrakeMode){
            headingPIDF = new asmPIDFController(Kp, Ki, Kd,Kf);
            headingPIDF.setSetPoint(targetHeading);
            headingPIDF.setTolerance(0.5);

            Lfront = hardwareMap.get(DcMotor.class,"lfd");
            Rfront = hardwareMap.get(DcMotor.class,"rfd");
            Lback = hardwareMap.get(DcMotor.class,"lbd");
            Rback = hardwareMap.get(DcMotor.class,"rbd");

            Lfront.setDirection(DcMotorSimple.Direction.FORWARD);
            Rfront.setDirection(DcMotorSimple.Direction.REVERSE);
            Lback.setDirection(DcMotorSimple.Direction.FORWARD);
            Rback.setDirection(DcMotorSimple.Direction.REVERSE);

    //        Lfront.setRunMode(Motor.RunMode.VelocityControl);
    //        Rfront.setRunMode(Motor.RunMode.VelocityControl);
    //        Lback.setRunMode(Motor.RunMode.VelocityControl);
    //        Rback.setRunMode(Motor.RunMode.VelocityControl);
    //
    //        Lfront.setFeedforwardCoefficients(kS,kV,kA);
    //        Rfront.setFeedforwardCoefficients(kS,kV,kA);
    //        Lback.setFeedforwardCoefficients(LBackkS,LBackkV,LBackkA);
    //        Rback.setFeedforwardCoefficients(kS,kV,kA);
    //
    //        Lfront.setVeloCoefficients(KP, KI, KD);
    //        Rfront.setVeloCoefficients(KP, KI, KD);
    //        Lback.setVeloCoefficients(LBackKP, LBackKI, LBackKD);
    //        Rback.setVeloCoefficients(KP, KI, KD);

            if(useBrakeMode){
                Lfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Rfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Lback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Rback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }else{
                Lfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Rfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Lback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Rback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }




            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.initialize();
            pinpoint.resetDeviceConfigurationForOpMode();
            pinpoint.recalibrateIMU();
            pinpoint.resetPosAndIMU();
            pinpoint.setPosition(new Pose2D(DistanceUnit.CM,0,0,AngleUnit.RADIANS,0));
//            pinpoint.setOffsets();
        }

        public void update(double leftX,double leftY,double rightX, double turnCoeff,boolean headingToTarget,boolean isRobotCentric){
            pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

            if(isRobotCentric){
                driveFieldCentric(
                        leftX,
                        leftY,
                        rightX /turnCoeff,
                        0,
                        headingToTarget
                );
            }else{
                driveFieldCentric(
                        leftX,
                        leftY,
                        rightX /turnCoeff,
                        pinpoint.getHeading(AngleUnit.RADIANS),
                        headingToTarget
                );
            }

        }



        public void driveFieldCentric(double strafeSpeed, double forwardSpeed,
                                      double turnSpeed, double gyroAngle,boolean headingLockEnabled) {

            strafeSpeed = clipRange(strafeSpeed);
            forwardSpeed = clipRange(forwardSpeed);
            turnSpeed = clipRange(turnSpeed);

            // --------------------------HEADINGLOCK----------------------
            if (headingLockEnabled) {
                headingPIDF.setSetPoint(targetHeading);
                double correction = headingPIDF.calculate(pinpoint.getHeading(AngleUnit.RADIANS));

                turnSpeed = correction;
            }

            double rotatedStrafe = strafeSpeed * Math.cos(-gyroAngle) - forwardSpeed * Math.sin(-gyroAngle);
            double rotatedForward = strafeSpeed * Math.sin(-gyroAngle) + forwardSpeed * Math.cos(-gyroAngle);

            
            double theta = Math.atan2(rotatedForward, rotatedStrafe);
            double magnitude = Math.sqrt(rotatedStrafe * rotatedStrafe + rotatedForward * rotatedForward);

            double[] wheelSpeeds = new double[4];
            wheelSpeeds[0] = Math.sin(theta + Math.PI / 4);
            wheelSpeeds[1] = Math.sin(theta - Math.PI / 4);
            wheelSpeeds[2] = Math.sin(theta - Math.PI / 4);
            wheelSpeeds[3] = Math.sin(theta + Math.PI / 4);

            normalizeWithMagnitude(wheelSpeeds, magnitude);

            wheelSpeeds[0] += turnSpeed;
            wheelSpeeds[1] -= turnSpeed;
            wheelSpeeds[2] += turnSpeed;
            wheelSpeeds[3] -= turnSpeed;

            normalize(wheelSpeeds);

            driveWithMotorPowers(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
        }

        private double clipRange(double value) {
            return Math.max(-1.0, Math.min(1.0, value));
        }

        private void normalizeWithMagnitude(double[] wheelSpeeds, double magnitude) {
            double maxMagnitude = 0;
            for (double speed : wheelSpeeds) {
                maxMagnitude = Math.max(maxMagnitude, Math.abs(speed));
            }

            if (maxMagnitude > 0) {
                for (int i = 0; i < wheelSpeeds.length; i++) {
                    wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
                }
            }
        }

        private void normalize(double[] wheelSpeeds) {
            double maxMagnitude = 0;
            for (double speed : wheelSpeeds) {
                maxMagnitude = Math.max(maxMagnitude, Math.abs(speed));
            }

            if (maxMagnitude > 1.0) {
                for (int i = 0; i < wheelSpeeds.length; i++) {
                    wheelSpeeds[i] /= maxMagnitude;
                }
            }
        }
        public void setTargetHeading(){
            targetHeading = pinpoint.getHeading(AngleUnit.RADIANS);
            headingPIDF.setSetPoint(targetHeading);
        }
        public void resetHeading(){
            pinpoint.setHeading(0,AngleUnit.RADIANS);
            pinpoint.recalibrateIMU();
            pinpoint.resetPosAndIMU();
//            pinpoint.resetDeviceConfigurationForOpMode();
//            pinpoint.setPosition(new Pose2D(DistanceUnit.CM,0,0,AngleUnit.RADIANS,0));

        }

        private void driveWithMotorPowers(double frontLeft, double frontRight,
                                          double backLeft, double backRight) {
            Lfront.setPower(frontLeft);
            Rfront.setPower(frontRight);
            Lback.setPower(backLeft);
            Rback.setPower(backRight);
        }

        public void viewTelemetry(Telemetry telemetry){
            telemetry.addLine("====================BASE=========================");
            telemetry.addData("Target Heading",targetHeading);
            telemetry.addData("heading error: ",headingPIDF.getPositionError());
            telemetry.addData("velocity error: ",headingPIDF.getVelocityError());
            telemetry.addData("Angle in radian",pinpoint.getHeading(AngleUnit.RADIANS));
            telemetry.addData("Angle in degrees",pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Debug",pinpoint.getConnectionInfo());
            telemetry.addData("X",pinpoint.getPosX(DistanceUnit.CM));
            telemetry.addData("Y",pinpoint.getPosY(DistanceUnit.CM));
        }
    }
