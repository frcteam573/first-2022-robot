#include "Robot.h"
#include "Log.h"
#include "frc/PowerDistribution.h"

using namespace std;

int counter;



Log::Log() {
    // Define CAN and PWM Ids used in Led here

    Board = new frc::PowerDistribution(1,frc::PowerDistribution::ModuleType::kRev);
    // Define motors, sensors, and pneumatics here

}

void Log::Dashboard (){

    double val;
    val=Board->GetCurrent(3);
    frc::SmartDashboard::PutString("Intake",to_string(val));

    val=Board->GetCurrent(11);
    frc::SmartDashboard::PutString("Pre Feed",to_string(val));

    val=Board->GetCurrent(6);
    frc::SmartDashboard::PutString("Shooter Feed",to_string(val));
    
    val=Board->GetCurrent(4);
    frc::SmartDashboard::PutString("Left Climber",to_string(val));

    val=Board->GetCurrent(5);
    frc::SmartDashboard::PutString("Right Climber",to_string(val));

    val=Board->GetCurrent(7);
    frc::SmartDashboard::PutString("Right Front Drive",to_string(val));

    val=Board->GetCurrent(8);
    frc::SmartDashboard::PutString("Right Back Drive",to_string(val));

    val=Board->GetCurrent(9);
    frc::SmartDashboard::PutString("Left Front Drive",to_string(val));

    val=Board->GetCurrent(10);
    frc::SmartDashboard::PutString("Left Back Drive",to_string(val));

}

void Log::PDPTotal(){
	double val = Board -> GetTotalCurrent();
	bool light;


	if (val > 250){

		counter = counter + 1;
		if (counter > 50){
			light = true;
		}
		else{
			light = false;
		}
	}
	else {
		counter = 0;
		light = false;
	}

	frc::SmartDashboard::PutBoolean("Over 250 Amps", light);
	//auto Gyrooutstr = std::to_string(counter);
	//frc::SmartDashboard::PutString("DB/String 5",Gyrooutstr);
}

void Log::CurrentCompare(int slot,double PWMin){

	double current = Board -> GetCurrent(slot);

	if (abs(PWMin) > .2){
		if (abs(current) < 1){
			
			frc::SmartDashboard::PutString("No Current Draw",to_string(slot));
		}
	}
}