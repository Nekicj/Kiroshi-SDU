package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Utils.asmConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * хэ
 * Я успешнее тебя
 * пока ты копишь на Off-White, пока я трачу на Goyard
 * хэй
 * Я успешнее тебя
 * Ведь ты своим дерьмом не заработал ни рубля
 * Я успешнее тебя
 * Я как Саня Богданов, у меня так много яхт d
 * Я успешнее тебя
 * Все твои кореша хотят попасть в Кузнецкий Сквад (Сквад)
 */

@Autonomous(name="      ")
public class Auto21Close extends OpMode {


    private Follower follower;
    private boolean isBlue = false;

    private PathChain START_TO_SCORE,SCORE_TO_TAKE2_N_GATE = null;

    @Override
    public void init(){
        isBlue = asmConfig.isBlue;

        follower = Constants.createFollower(hardwareMap);
        if(isBlue){
            follower.setStartingPose(FieldConstants.Blue.Start.CLOSE);

            START_TO_SCORE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.Start.CLOSE,FieldConstants.Blue.SCORE_POSE))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Start.CLOSE.getHeading(),FieldConstants.Blue.SCORE_POSE.getHeading(),0.8)
                    .build();

            SCORE_TO_TAKE2_N_GATE = follower.pathBuilder()
                    .addPath(new BezierLine(FieldConstants.Blue.SCORE_POSE,FieldConstants.Blue.Take2Gate.START))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.SCORE_POSE.getHeading(),FieldConstants.Blue.Take2Gate.START.getHeading(),0.8)
                    .setTValueConstraint(0.85)

                    .addPath(new BezierLine(FieldConstants.Blue.Take2Gate.START,FieldConstants.Blue.Take2Gate.FINAL))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take2Gate.START.getHeading(),FieldConstants.Blue.Take2Gate.FINAL.getHeading(),0.8)
                    .setTValueConstraint(0.85)

                    .addPath(new BezierLine(FieldConstants.Blue.Take2Gate.FINAL,FieldConstants.Blue.SCORE_POSE))
                    .setLinearHeadingInterpolation(FieldConstants.Blue.Take2Gate.FINAL.getHeading(),FieldConstants.Blue.SCORE_POSE.getHeading(),0.8)
                    .setTValueConstraint(0.85)

                    .build();


        }
    }

    @Override
    public void loop(){

    }
}