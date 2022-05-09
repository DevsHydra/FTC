package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

@Autonomous
public class Set_Gravacao_Test<Servo> extends LinearOpMode {
    private DcMotor LMF;
    private DcMotor LMB;
    private DcMotor RMF;
    private DcMotor RMB;
    private com.qualcomm.robotcore.hardware.Servo SC;
    private DcMotor Esteira;
    double valorCarregado[][] = new double[500][4];
    double timer = 0;
    double timer2 = 0;
    double jGravacao = 0;
    @Override

    public void runOpMode() throws InterruptedException {
        try {
            this.loadGravacao("/storage/emulated/0/FIRST/RecordedGravacao/AutonomoGeduc.txt");
        } catch (IOException e) {
            e.printStackTrace();
        }
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        LMB = hardwareMap.get(DcMotor.class, "LMB");
        Esteira = hardwareMap.get(DcMotor.class, "MMT");
        SC = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "SC");

        // Set Direction
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
        SC.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);

        RMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        SC.setPosition(1);
        sleep(1000);
        allPower(0.5);
        sleep(2000);
        powerDouble(0.5,-0.5);
        sleep(2000);

    }


    private void loadGravacao(String pathName) throws IOException {
        BufferedReader reader = new BufferedReader(new FileReader(pathName));
        String line = "";
        int row = 0;
        while((line = reader.readLine()) != null) {
            String[] cols = line.split(",");
            int col = 0;
            for(String c : cols) {
                valorCarregado[row][col] = Double.parseDouble(c);
                col++;
            }
            row++;
            jGravacao = row;
        }
        reader.close();
    }
    public void setPowerGravado() {
//
//        timer2 = System.currentTimeMillis();
//        while (System.currentTimeMillis() - timer2 < 10000) {
//            if (System.currentTimeMillis() - timer < 20) {
//                continue;
//            }
//            timer = System.currentTimeMillis();
            for (int i = 0; i < jGravacao; i++) {
                telemetry.addLine("SET POWER GRAVADOR");
                telemetry.addData("valor armazenado: ", valorCarregado[i][0]);
                RobotLog.d("valor_armazenado_RMF: " + i + " Valor" + valorCarregado[i][0]);
                telemetry.update();
                RMF.setPower(valorCarregado[i][0]);
                LMF.setPower(valorCarregado[i][1]);
                RMB.setPower(valorCarregado[i][2]);
                LMB.setPower(valorCarregado[i][3]);
                sleep(20);
            }
    }

    public void allPower(double force){
        RMF.setPower(force);
        LMF.setPower(force);
        RMB.setPower(force);
        LMB.setPower(force);
    }
    public void powerDouble(double forceD, double forceE){
        RMF.setPower(forceD);
        LMF.setPower(forceE);
        RMB.setPower(forceE);
        LMB.setPower(forceD);
    }

}
