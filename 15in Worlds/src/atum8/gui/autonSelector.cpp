#include "atum8/gui/autonSelector.hpp"
#include "main.h"

namespace atum8 {

void AutonSelector::scrollRight() {
  if (lcdCurrent == lcdMax) {
    lcdCurrent = lcdMax;
  } else {
    lcdCurrent++;
    pros::delay(200);
  }
}

void AutonSelector::scrollLeft() {
  if (lcdCurrent == lcdMin) {
    lcdCurrent = lcdMin;
  } else {
    lcdCurrent--;
    pros::delay(200);
  }
}

void AutonSelector::confirmSelection(){
    program = lcdCurrent;
	pros::lcd::clear_line(1);
	pros::lcd::clear_line(7);
	pros::lcd::set_text(2, "         Autonomous Has");
	pros::lcd::set_text(3, "         Been Selected!");
	pros::delay(1500);
}

void AutonSelector::displayRedSelection(){
    pros::lcd::set_text(1, "Alliance Color: Red");
	pros::lcd::set_text(2, "Purpose: Matches");
	pros::lcd::set_text(3, "Points: 60");
	pros::lcd::set_text(7, "Curent State: Selection");
}

void AutonSelector::displayRedActive(){
    pros::lcd::set_text(1, "Alliance Color: Red");
	pros::lcd::set_text(2, "Purpose: Matches");
	pros::lcd::set_text(3, "Points: 60");
	pros::lcd::set_text(7, "Curent State: Active");
}

void AutonSelector::displayBlueSelection(){
    pros::lcd::set_text(1, "Alliance Color: Blue");
	pros::lcd::set_text(2, "Purpose: Matches");
	pros::lcd::set_text(3, "Points: 60");
	pros::lcd::set_text(7, "Curent State: Selection");
}

void AutonSelector::displayBlueActive(){
    pros::lcd::set_text(1, "Alliance Color: Blue");
	pros::lcd::set_text(2, "Purpose: Matches");
	pros::lcd::set_text(3, "Points: 60");
	pros::lcd::set_text(7, "Curent State: Active");
}

void AutonSelector::displaySkillsSelection(){
    pros::lcd::set_text(1, "Alliance Color: Red");
	pros::lcd::set_text(2, "Purpose: Skills");
	pros::lcd::set_text(3, "Points: 220");
	pros::lcd::set_text(7, "Curent State: Selection");
}

void AutonSelector::displaySkillsActive(){
    pros::lcd::set_text(1, "Alliance Color: Red");
	pros::lcd::set_text(2, "Purpose: Skills");
	pros::lcd::set_text(3, "Points: 220");
	pros::lcd::set_text(7, "Curent State: Active");
}

void AutonSelector::displayTestingSelection(){
    pros::lcd::set_text(1, "Alliance Color: Purple idk");
	pros::lcd::set_text(2, "Purpose: Testing");
	pros::lcd::set_text(3, "Points: YUP");
	pros::lcd::set_text(7, "Curent State: Selection");
}

void AutonSelector::displayTestingActive(){
    pros::lcd::set_text(1, "Alliance Color: Purple idk");
	pros::lcd::set_text(2, "Purpose: Testing");
	pros::lcd::set_text(3, "Points: YUP");
	pros::lcd::set_text(7, "Curent State: Active");
}


void AutonSelector::selector() {

    if (pros::lcd::read_buttons() == LCD_BTN_RIGHT)
      scrollRight();
    else if (pros::lcd::read_buttons() == LCD_BTN_LEFT)
      scrollLeft();

    // Red Autonomous Selection Screen
	if(lcdCurrent == 1 && program != 1){
        displayRedSelection();
		if(pros::lcd::read_buttons() == LCD_BTN_CENTER)
            confirmSelection();
    }
	else if(lcdCurrent == 1 && program == 1){
        displayRedActive();
    }

    // Blue Autonomous Selection Screen
	if(lcdCurrent == 2 && program != 2){
        displayBlueSelection();
		if(pros::lcd::read_buttons() == LCD_BTN_CENTER)
            confirmSelection();
    }
	else if(lcdCurrent == 2 && program == 2){
        displayBlueActive();
    }

    // Skills Autonomous Selection Screen
	if(lcdCurrent == 3 && program != 3){
        displaySkillsSelection();
		if(pros::lcd::read_buttons() == LCD_BTN_CENTER)
            confirmSelection();
    }
	else if(lcdCurrent == 3 && program == 3){
        displaySkillsActive();
    }

    // Testing Autonomous Selection Screen
	if(lcdCurrent == 4 && program != 4){
        displayTestingSelection();
		if(pros::lcd::read_buttons() == LCD_BTN_CENTER)
            confirmSelection();
    }
	else if(lcdCurrent == 4 && program == 4){
        displayTestingActive();
    }
}
} // namespace atum8