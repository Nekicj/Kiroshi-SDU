package org.firstinspires.ftc.teamcode.OpModes.Switchers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Utils.asmConfig;
@Disabled
@TeleOp(name="GPP switch",group = "Switchers")
public class switchToGPP extends LinearOpMode {

    @Override
    public void runOpMode(){
        asmConfig.pattern = 0;
    }
}
