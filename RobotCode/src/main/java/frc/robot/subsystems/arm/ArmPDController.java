package frc.robot.subsystems.arm;


public class ArmPDController {

    double kP, kD;

    double pSetpoint, dSetpoint;


    public ArmPDController(double kP, double kD) {
        this.kP=kP;
        this.kD=kD;
    }

    public void setSetpoint(double pSetpoint, double dSetpoint) {
        this.pSetpoint = pSetpoint;
        this.dSetpoint = dSetpoint;
    }

    public double calculate(double pMeasurement, double dMeasurement) {
        return kP*(pSetpoint-pMeasurement) + kD*(dSetpoint-dMeasurement);
    }

    public double calculate(double pMeasurement, double dMeasurement, double pSetpoint, double dSetpoint) {
        this.pSetpoint = pSetpoint;
        this.dSetpoint = dSetpoint;

        return kP*(pSetpoint-pMeasurement) + kD*(dSetpoint-dMeasurement);
    }
}