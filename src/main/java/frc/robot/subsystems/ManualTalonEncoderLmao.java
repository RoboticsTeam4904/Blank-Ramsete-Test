package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManualTalonEncoderLmao {
    public WPI_TalonFX talon;
    private double distancePerPulse = 1; // TODO make sure to override
    private double offset = 0;
    private int id;
    public ManualTalonEncoderLmao(WPI_TalonFX talon, int id) {
        this.talon = talon;
        this.id = id;
    }

    public void setDistancePerPulse(double distancePerPulse) {
        this.distancePerPulse = distancePerPulse;
    }

    public void debug() {
        SmartDashboard.putNumber("gaming " + id, talon.getSelectedSensorPosition());
        SmartDashboard.putNumber("d/dx(gaming) " + id, talon.getSelectedSensorVelocity());

        SmartDashboard.putNumber("is inverted? " + id, java.lang.Math.signum(talon.getSelectedSensorVelocity()));
    }

    public double getRate() {
        return talon.getSelectedSensorVelocity() * distancePerPulse;
    }

    public double getDistance() {
        return (talon.getSelectedSensorPosition() - offset) * distancePerPulse;
    }
    
    public void reset() {
        offset = talon.getSelectedSensorPosition();
    }
}
