package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Controle2 extends LinearOpMode {
    private DcMotor MDP = null;     //Motor - Derrubar Pato
    private DcMotor MC = null;      //Motor - Coletar
    private Servo SG = null;        //Servo - Garra
    private Servo SB = null;        //Servo - Braço
    private DcMotor MB = null;      //Motor - Braço - Ticks 1120

    @Override
    public void runOpMode() throws InterruptedException {
        double[] powerYJoy2 = {-gamepad2.left_stick_y,-gamepad2.right_stick_y};   //[0] - Coleta | [1] - Derrubar pato
        boolean povD = gamepad2.dpad_right;
        boolean povE = gamepad2.dpad_left;
        boolean povUp = gamepad2.dpad_up;
        boolean toggle = true;
        boolean antPov = false;
        boolean qqw = true;
        double ticksBraco = 1120;
        double integral = 0, erro = 0, somatorioErro = 0, ki = 0;
        int pointBraco = 0;
        double powerBraco = 0;
        String nivel = "";

        MC = hardwareMap.get(DcMotor.class,"MC");
        MDP = hardwareMap.get(DcMotor.class, "MDP");
        SG = hardwareMap.get(Servo.class, "SG");
        MB = hardwareMap.get(DcMotor.class, "MB");

        // POV
//        DIREITA 90 GRAUS
//        ESQUERDA -90
//        CIMA 0


        MB.setDirection(DcMotorSimple.Direction.FORWARD);
        MC.setDirection(DcMotorSimple.Direction.FORWARD);
        MDP.setDirection(DcMotorSimple.Direction.FORWARD);
        SG.setDirection(Servo.Direction.REVERSE);

        MB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//
//        if(Braco.getCurrentPosition() != pointBraco){
//            Braco.setPower(0.5);
//            Braco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        }


        waitForStart();
        while(opModeIsActive()){

            // SET POSICAO SERVO
            if (povD && !antPov){
                SG.setPosition(toggle ? 1 : 0);
                toggle = !toggle;
                antPov = true;
            } else if (!povD){
                antPov = false;
            }

            // SET POWER MOTORS
            MC.setPower(powerYJoy2[0]);




            
            MDP.setPower(powerYJoy2[1]);

            telemetry.addData("Encoder on/off:", MB.isBusy());
            telemetry.addData("Posição que o braço deve ir:", MB.getTargetPosition());
            telemetry.addData("Posicao atual do Braco: ", MB.getCurrentPosition());
            telemetry.update();




            //Alinhamento CAIXA - BRAÇO
            telemetry.addData("Controlador Servo: ",SB.getController());
            telemetry.addData("Posição Servo:", SB.getPosition());
            telemetry.addData("Direção do Servo", SB.getDirection());
            telemetry.addData("Numero da Porta:", SB.getPortNumber());


            //Niveis do Braço

            //nivel = gamepad1.a ? "Coletar" : gamepad1.b ? "3" : gamepad1.x ?  "1" : gamepad2.y ? "2" : !gamepad2.a && !gamepad2.b && !gamepad2.x && !gamepad2.y ? "NA";

            if(gamepad2.a && gamepad2.left_bumper && gamepad2.right_stick_button){
                qqw = false;
                MB.setPower(-gamepad2.left_stick_y/6);
                MB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            while (qqw) {
                if (gamepad2.a) {
                    nivel = "Coletar";
                } else if (gamepad2.b) {
                    nivel = "3";
                } else if (gamepad2.y) {
                    nivel = "2";
                } else if (gamepad2.x) {
                    nivel = "1";
                } else {
                    nivel = "NA";
                }


                if (nivel == "Coletar") {
                    pointBraco = 0;
                } else if (nivel == "1") {
                    pointBraco = -969;
                } else if (nivel == "2") {
                    pointBraco = -885;
                } else if (nivel == "3") {
                    pointBraco = -755;
                } else if (nivel == "NA") {
                    pointBraco = -10;
                }

                powerBraco = 0.5; // Dps irei fazer uma prporcional relacionando a força em função da distancia do setPoint

                if (MB.getCurrentPosition() != pointBraco) {
                    MB.setTargetPosition(pointBraco);
                    MB.setPower(powerBraco);
                    MB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else {
                    MB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
            qqw = true;

        }
    }
}
//Se usando apenas esse if não funcionar, aqui a integral
            /*
            if(MB.getCurrentPosition() != pointBraco){
                erro = pointBraco - MB.getCurrentPosition();
                integral = ki * somatorioErro;
                somatorioErro += erro;

                MB.setPower(integral);
            }
            */
