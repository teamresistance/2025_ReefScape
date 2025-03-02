package frc.robot.subsystems.vision;
import edu.wpi.first.math.trajectory.Trajectory;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class SwerveModule {
    int driveMotor_ID;
    int turningMotor_ID;
    int turningEncoder_ID;
    SparkMax driveMotor;
    SparkMax turningMotor;
    public SwerveModule(int driveMotorID, int turningMotorID, int turningEncoderID) { 
        driveMotor_ID= driveMotorID; 
        turningMotor_ID = turningMotorID;
        turningEncoder_ID = turningEncoderID;
        SparkMax driveMotor= new SparkMax(driveMotor_ID, MotorType.kBrushless);
        SparkMax turningMotor= new SparkMax(turningMotor_ID, MotorType.kBrushless);     

    }
    public void setDesiredState(Trajectory.State desiredState){
        //set the desired state of the swerve module
        driveMotor.set(desiredState.velocityMetersPerSecond);
        turningMotor.set(desiredState.curvatureRadPerMeter);

    }
}
