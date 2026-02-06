# SECON26 - IEEE SoutheastCon 2026 Hardware Competition
Repository for the Florida Polytechnic's IEEE SoutheastCon 2026 Hardware Competition, showcasing an autonomous robot and micro UAV system designed for the Lunar Rescue Mission set in the year 2075. This project details the development of a system engineered to compete in a lunar themed challenge involving rescue and communication tasks.

## Competition Overview
The Lunar Rescue Mission, set in the year 2075, forms the theme of the IEEE SoutheastCon 2026 Hardware Competition. The challenge requires participants to rescue six stranded Astro Ducks from the lunar surface, restore power to four antennas through unique tasks, establish communication with Earth using a photodiode, and transmit the LED colors of the antennas to Earth.

## System Architecture

### Ground Robot
The ground robot serves as the primary system for executing the lunar rescue and antenna tasks. It is controlled by a Raspberry Pi running Embeed C. The robot has a maximum weight of 25 pounds and has to be starting in an area of 12 inches by 12 inches by 12 inches. The robot's actuators consist of motors for mobility and servo mechanisms for performing antenna tasks.

### Micro UAV
An optional micro UAV complements the system by handling specific tasks. Controlled by an ESP32 microcontroller with Rust and C based firmware, the UAV has a maximum weight of 250 grams (0.55 pounds). Its primary functions include detecting LED colors on the antennas and transmitting data to Earth via infrared (IR) communication to a photodiode.

## Competition Tasks
### Antenna Tasks
To power on the four antennas, the robot must complete distinct tasks for each. The first antenna requires pressing a button three times. The second antenna involves rotating a crank 540 degrees, either clockwise or counterclockwise. The third antenna task entails removing an Astro-Duck from a pressure plate. The fourth antenna requires entering the keypad code "73738#" (RESET).

### Scoring Opportunities
The competition awards points based on task completion. Exiting the starting area earns 20 points. Powering on each of the four antennas grants 15 points per antenna, totaling 60 points. Returning each of the six Astro Ducks to Echo Base yields 5 points per duck, for a total of 30 points. Establishing the first Earth connection is worth 20 points. Transmitting the correct LED color for each of the four antennas awards 30 points per antenna, totaling 120 points. Successfully navigating craters can earn up to 85 points. Launching and retrieving the UAV contributes 80 points. The maximum achievable score is 415 points.

## Team Members
- Joshua Gottus - Software Lead
- Nathan Piel - Software Lead

## License
This project is licensed under the MIT License. See the LICENSE file for details.

---

| Field                 | Details                          |
|-----------------------|----------------------------------|
| IEEE Student Branch   | Florida Polytechnic University   |
| Region                | IEEE Region 3 (SoutheastCon)     |
| Competition Location  | Alabama                          |
