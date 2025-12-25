    package org.firstinspires.ftc.teamcode.Controllers;

    import com.acmerobotics.dashboard.config.Config;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.hardware.VoltageSensor;

    import org.firstinspires.ftc.robotcore.external.Telemetry;

    @Config
    public class ShooterControllerPIDVSA {
        private DcMotorEx shooterMotorLeft;
        private DcMotorEx shooterMotorRight;
        private Servo servor = null;
        private VoltageSensor voltageSensor;

        private double directionPos = 0.7;

//        public static double kS = 1.1;    // Volts - static friction
//        public static double kV = 0.0048;  // Volts per RPM - velocity constant
//        public static double kA = 0.001;  // Volts per RPM/s - acceleration constant
//
//        public static double kP = 0.025;
//        public static double kI = 0.0;
//        public static double kD = 0;

        public static double kS = 0.9;    // Volts - static friction
        public static double kV = 0.0052;  // Volts per RPM - velocity constant
        public static double kA = 0.0003;  // Volts per RPM/s - acceleration constant

        public static double kP = 0.035;
        public static double kI = 0.00005;
        public static double kD = 0.0015;



        private double targetVelocityRPM = 0;
        private double lastError = 0;
        private double integral = 0;
        private double lastVelocity = 0;
        private long lastTime = 0;

        private static final double NOMINAL_VOLTAGE = 12.5;

        public static double servoClose = 0.3;
        public static double servoMedium = 0.45;
        public static double servoHigh = 0.63;

        public static enum ServosPos{
            DIRECTION_DOWN(0.6),
            DIRECTION_UP(0.3);


            private final double position;
            ServosPos(double pos) {this.position = pos;}


            public double getPos() {return position;}
        }

        public void initialize(HardwareMap hardwareMap, String shooterMotorLeftName, String shooterMotorRightName, String servoAngleRightName,double pos) {
            shooterMotorLeft = hardwareMap.get(DcMotorEx.class, shooterMotorLeftName);
            shooterMotorRight = hardwareMap.get(DcMotorEx.class, shooterMotorRightName);

            servor = hardwareMap.get(Servo.class,servoAngleRightName);
            servor.setDirection(Servo.Direction.REVERSE);

            servor.setPosition(pos);

            voltageSensor = hardwareMap.voltageSensor.iterator().next();

            shooterMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
            shooterMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);

            lastTime = System.nanoTime();
        }
        public boolean checkVelocity(double targetVelocity,double offset){
            if(Math.abs(shooterMotorLeft.getVelocity()) >= Math.abs(targetVelocity -offset) ){
                return true;
            }else{
                return false;
            }
        }

        public void setShooterVelocity(double targetRPM) {
            this.targetVelocityRPM = targetRPM;
        }
        public void setDirectionPos(double setPos){
            directionPos = setPos;
            powDirectionPos();
        }

        public void powDirectionPos(){
            servor.setPosition(directionPos);
        }
        public void update() {
            if (targetVelocityRPM == 0) {
                shooterMotorLeft.setPower(0);
                shooterMotorRight.setPower(0);
                integral = 0;
                lastError = 0;
                return;
            }

            long currentTime = System.nanoTime();
            double dt = (currentTime - lastTime) * 1e-9;
            lastTime = currentTime;

            double currentVelocity = (shooterMotorLeft.getVelocity() + shooterMotorRight.getVelocity()) / 2.0;

            double acceleration = (currentVelocity - lastVelocity) / dt;
            lastVelocity = currentVelocity;

            double currentVoltage = voltageSensor.getVoltage();
    //        double voltageCompensation = NOMINAL_VOLTAGE / currentVoltage;

            double feedforwardVoltage = kS * Math.signum(targetVelocityRPM) +
                    kV * targetVelocityRPM +
                    kA * acceleration;

            double error = targetVelocityRPM - currentVelocity;
            integral += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;

            double pidVoltage = kP * error + kI * integral + kD * derivative;

            double totalVoltage = feedforwardVoltage + pidVoltage;

            double power = totalVoltage / currentVoltage;

    //        power *= voltageCompensation;

            power = Math.max(-1.0, Math.min(1.0, power));

            shooterMotorLeft.setPower(power);
            shooterMotorRight.setPower(power);
        }

        public boolean isReadyToShoot(double tolerance) {
            if (targetVelocityRPM == 0) return false;
            double currentVelocity = (shooterMotorLeft.getVelocity() + shooterMotorRight.getVelocity()) / 2.0;
            return Math.abs(currentVelocity - targetVelocityRPM) < tolerance;
        }




        public void showTelemetry(Telemetry telemetry) {
            double currentVelocity = (shooterMotorLeft.getVelocity() + shooterMotorRight.getVelocity()) / 2.0;
            double currentVoltage = voltageSensor.getVoltage();
            double voltageCompensation = NOMINAL_VOLTAGE / currentVoltage;

            telemetry.addLine("=== WPILib Shooter Controller ===");
            telemetry.addData("Target RPM", "%.1f", targetVelocityRPM);
            telemetry.addData("Current RPM", "%.1f", currentVelocity);
            telemetry.addData("Error RPM", "%.1f", targetVelocityRPM - currentVelocity);
            telemetry.addData("Battery Voltage", "%.2fV", currentVoltage);
            telemetry.addData("Voltage Compensation", "%.3f", voltageCompensation);
            telemetry.addData("Ready to Shoot", isReadyToShoot(50) ? "YES" : "NO");
            telemetry.addData("Motor Power", "%.3f", shooterMotorLeft.getPower());
        }
    }