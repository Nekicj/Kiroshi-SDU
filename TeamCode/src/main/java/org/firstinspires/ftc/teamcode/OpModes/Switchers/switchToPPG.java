package org.firstinspires.ftc.teamcode.OpModes.Switchers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.asmConfig;
@Disabled
@TeleOp(name="PPG switch",group = "Switchers")
public class switchToPPG extends LinearOpMode {

    @Override
    public void runOpMode(){
        asmConfig.pattern = 2;
    }
}
