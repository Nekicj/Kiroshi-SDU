package org.firstinspires.ftc.teamcode.Kotak.TestOpModes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="limelight test",group = "limelight")
public class LimeLightTest extends OpMode {
    private Limelight3A limelight;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class,"limelight");

        limelight.pipelineSwitch(0);


    }
    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop(){
        LLResult llResult =  limelight.getLatestResult();
        if(llResult != null && llResult.isValid()){
            double tx=llResult.getTx();
            telemetry.addData("tX: ",tx);
            telemetry.addData("lgihtl",llResult.getDetectorResults());
            telemetry.update();

        }



    }
}
