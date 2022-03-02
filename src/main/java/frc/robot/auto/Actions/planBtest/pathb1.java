// package frc.robot.auto.Actions.planBtest.Drive;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Drive;

// public class pathb1 extends CommandBase {
//   private Drive drive;
//   private double position;
//   private double target = 0.9;
//   private double error = target - position;
//   private double currentangle ;
//   private double targetangle  = 124;
//   private double turnrate = targetangle - currentangle;

//   public pathb1(Drive m_drive) {
//     drive = m_drive;
//     addRequirements(drive);
//   }

//   @Override
//   public void initialize() {
//     drive.resetEncoders();
//     drive.zeroHeading();
//   }

//   @Override
//   public void execute() {
//     position = drive.getLinearEncoderPosition();
//     currentangle = drive.getHeading();
//     if(error > 0){
//       drive.drivePolar( - error * 0.2 , 0 , 0);
//     }
//     else if(error == 0){
//       drive.drivePolar(0, 0, 0);
//       drive.resetEncoders();
//       target = 2.62 ;
//     else if(targetangle != 0){
//       drive.drivePolar(0 , 0 , turnrate * 0.2);
//     }
//     else if (error > 0){
//       drive.drivePolar( - error * 0.2 , 0 , 0);
//     }
//     else if (error == 0){
//       drive.drivePolar(0, 0 , 0);
//       drive.resetEncoders();
//       target = 3.68;
//       targetangle = 43;
//     }
//     else if (targetangle != 0){
//         drive.drivePolar(0, 0, turnrate * 0.2);
//     }
//     else if(error > 0){
//       drive.drivePolar( - error * 0.2, 0 , 0);
//     }
//     else if (error == 0){
//       drive.drivePolar(0, 0 , 0);
//     }
//   }
// }