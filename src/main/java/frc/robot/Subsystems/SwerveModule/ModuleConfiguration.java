package frc.robot.Subsystems.SwerveModule;


public interface ModuleConfiguration {

    //any swerve version should extend off of SwerveModuleBase which
    //implements ModuleConfig.
    //these methods are 'universal' in the sense that each module type MUST INCLUDE
    //these, but the return/methods contain different motors/encoders.
    double getAbsPosition();
    double getModuleVelocity();
    double getAngularModuleVelocity();
    double getDistanceTravelled();
    void configureSettings();
    void setModule(double drive, double turn);
}
