package frc.lib;

public class MotorTempObject {
    private String name;
    private double motorTemp;

    public MotorTempObject(String name, double motorTemp) {
        this.name = name;
        this.motorTemp = motorTemp;
    }

    public String getName() {
        return this.name;
    }
    public double getMotorTemp() {
        return this.motorTemp;
    }

    public String[] toStringArray() {
        return new String[] {
            "Name: " + name,
            "Motor Temperature: " + motorTemp + "°C",
        };
    }
}