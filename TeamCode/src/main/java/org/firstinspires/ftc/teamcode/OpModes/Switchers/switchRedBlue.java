package org.firstinspires.ftc.teamcode.OpModes.Switchers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.asmConfig;


@TeleOp(name="RedBlue Switch",group = "Switchers")
public class switchRedBlue extends LinearOpMode {

    @Override
    public void runOpMode(){
        asmConfig.isBlue = !asmConfig.isBlue;
    }
}
