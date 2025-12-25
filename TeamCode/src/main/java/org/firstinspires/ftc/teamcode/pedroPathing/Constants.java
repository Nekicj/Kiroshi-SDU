package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("lfd")
            .leftRearMotorName("lbd")
            .rightFrontMotorName("rfd")
            .rightRearMotorName("rbd")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(84.82118813822588)
            .yVelocity(68.42908147376353);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.1)
            .forwardZeroPowerAcceleration(-27.932741061626466)
            .lateralZeroPowerAcceleration(-69.67252519714293)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.065,
                    0,
                    0.015,
                    0.02
            ))
//            .translationalPIDFCoefficients(new PIDFCoefficients(
//                    0.08,dw3
//                    0,
//                    0.01,
//                    0.04
//            ))
            .translationalPIDFSwitch(2)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.06,
                    0,
                    0.004,                     //0.004,
                    0.04                //0.02
            ))
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
//                    0.4,   //0.4
//                    0,
//                    0.005,       //0.005
//                    0.0006
//            ))
            .useSecondaryTranslationalPIDF(true)
//            .headingPIDFCoefficients(new PIDFCoefficients(
//                    0.64,
//                    0,
//                    0.022,
//                    0.02
//            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.4,
                    0,
                    0.001,
                    0.08
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2, //2.5
                    0,
                    0.1,   //0.1
                    0.002 //0.0005
            ))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
//                    1, //2.5
//                    0,
//                    0.1,   //0.1
//                    0.02 //0.0005
//            ))
            .useSecondaryHeadingPIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.03,
                    0,
                    0,
                    0.6,
                    0.025
            ))
            .useSecondaryDrivePIDF(false)
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0,
                    0.000005,
                    0.6,
                    0.01
            ))

            .drivePIDFSwitch(15)
            .headingPIDFSwitch(0.2) //0.07
            .holdPointHeadingScaling(1)
            .holdPointTranslationalScaling(1)
            .turnHeadingErrorThreshold(0.005)
            .automaticHoldEnd(true)


            .centripetalScaling(0.0005);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.248)
            .strafePodX(-3.544)
            .hardwareMapName("pinpoint")
//            .yawScalar(1.000477)

            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)


            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            1,
            1,
            1,
            100,
            1.3,
            10,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}