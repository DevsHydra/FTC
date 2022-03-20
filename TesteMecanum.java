package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp (name =  "Teste Mecanum")

public class TesteMecanum extends LinearOpMode {
    private DcMotor LMF;    //Motor - Esquerda  Front
    private DcMotor LMB;    //Motor - Esquerda  Back
    private DcMotor RMF;    //Motor - Direita   Front
    private DcMotor RMB;    //Motor - Direita   Back
    private DcMotor MMT;    //Motor - Movimentação Tanque
    private DcMotor MDP;    //Motor - Derrubar Pato
    private DcMotor MC;     //Motor - Coletar

    boolean toggle = false;

    @Override
    public void runOpMode() {
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        MMT = hardwareMap.get(DcMotor.class, "MMT");
        MC = hardwareMap.get(DcMotor.class, "MC");
        MDP = hardwareMap.get(DcMotor.class, "MDP");

        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
        MMT.setDirection(DcMotorSimple.Direction.FORWARD);
        MC.setDirection(DcMotorSimple.Direction.FORWARD);
        MDP.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.left_stick_x * 1.5;
                double giro = Math.pow(gamepad1.right_stick_x, 3); //Exponenciação para arrumar sensibilidade
                double Redutor = 0;
                double[] poder = {0, 0, 0, 0};

                //Valores para movimentação com mechanum (lados espelhados)
                //Motor Esquerda Frente;
                poder[0] = drive + turn + giro;
                //Motor Esquerda trás;
                poder[1] = drive - turn + giro;
                //Motor Direita Frente;
                poder[2] = drive - turn - giro;
                //Motor Direita trás;
                poder[3] = drive + turn - giro;

                //Verificar se algum valor é maior que 1
                if (Math.abs(poder[0]) > 1 || Math.abs(poder[1]) > 1
                        || Math.abs(poder[2]) > 1 || Math.abs(poder[3]) > 1) {

                    //Achar o maior valor
                    double max;
                    max = Math.max(Math.abs(poder[0]), Math.abs(poder[1]));
                    max = Math.max(Math.abs(poder[2]), max);
                    max = Math.max(Math.abs(poder[3]), max);

                    //Não ultrapassar +/-1 (proporção);
                    poder[0] /= max;
                    poder[1] /= max;
                    poder[2] /= max;
                    poder[3] /= max;
                }

                RMF.setPower(poder[0]);
                RMB.setPower(poder[1]);
                LMB.setPower(poder[2]);
                LMF.setPower(poder[3]);
            }
        }
    }
}