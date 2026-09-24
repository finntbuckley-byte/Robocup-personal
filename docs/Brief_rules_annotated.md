# Brief_rules_annotated (text extract)

Text layer of `Brief_rules_annotated.pdf`. Use the PDF for figures/images.

<!-- page 1 -->
Classification: In-Confidence
2026 Robocup challenge – Objective, requirements, and rules
Note – these rules are subject to update or modification
Version 1.0 (23/02/2026) – changes since last version shown in red
Scenario: This year, the Robocup continues its Quidditchy
twist 
Objective: You have 2 minutes to gain the highest score by 
collecting target weights and/or snitches while avoiding 
dummy weights and bludgers using your mobile robot.
Competition: Two robots will be in the arena at the same 
time, collecting target weights for their team. There will be 
a number of weights in the arena for each round, as well as 
mobile elements (snitches and bludgers). 
Target weights and snitches add to your score. Dummy weights and bludgers subtract from 
your score.
Weights and mobile elements are considered collected, and thus contribute to your final 
score, if they are on board your robot at the end of the round. To make matters more 
interesting, at the end of the round, your robot can only have a maximum of 3 weights on
board. 
Target weights can be delivered back to your home base, and you receive a bonus for 
achieving this. 
At the end of the 2 minutes, the team with the highest score will be declared the winner.
The score for each round is calculated as: 
Score = 1x (weight of target weights on board robot) + 2x (weight of target weights in home 
base at the end of the round) + 3x (number of snitches on board robot) – 3x (number of 
bludgers on board robot) - 0.5x (number of dummy weights on board robot) – any penalties 
(see the fine print)
For example, if at the end of the round, your robot has:
▪ 0.75 kg target weight on board
▪ 3 kgs of target weights in your home base
▪ 1 snitch on board
▪ 1 dummy weight on board
Score = 1x 0.75 + 2x 3 + 3x 1 – 0.5x 1 = 9.25 points.

<!-- page 2 -->
Classification: In-Confidence
Environment: The arena in which the race takes place is 2.4 x 4.9m with 400mm high walls.
The arena will contain a number of obstacles that your robot will need to avoid or negotiate. 
The position of the obstacles for the final competition will not be revealed until the day of 
competition and will change regularly during the competition rounds, so your robot must be 
able to deal with these unknowns.
Within the arena, there will be a number of target weights (> 5) and dummy weights. The 
shape of all weights will be identical (i.e. A cylinder of diameter 50mm and height 70mm
with an annular groove to facilitate gripping) but vary in colour and weight. The location of 
the weights will vary and you will not know these positions ahead of time. 
There may also be up to 2 mobile elements (snitches and/or bludgers) consisting of plastic 
Sphero Bolt robots (https://www.sphero.com/sphero-bolt), approximately 73mm in diameter. 
They will roam the arena at up to 7 m/s and will be differentiated by the colour of their
illuminated LEDs: snitches will be illuminated yellow (gold) and bludgers red.
Within the arena there are two areas designated as bases. These areas will be coloured 
green and blue, different from the surrounding arena (black) and obstacles (red). These
bases will measure 600mm x 600mm and include a low rim to prevent weights rolling out. 
The robots will start each round on one of these bases. 
Hardware: You must construct your robot using:
Controller: Teensy 4.0 (ARM Cortex-M7
microcontroller). You cannot use other 
boards/controllers – but you won’t need to cos a 32-
bit processor running at up to 600 MHz is more than 
enough!
Chassis: You have the choice of using a supplied 
tracked chassis or building your own custom chassis 
from scratch.
Sensors/actuators/structure: You will be provided with:
i. A box of parts including various sensors, actuators, and structural elements such as 
aluminium plate, rods, acrylic etc.
ii. A budget of $50 which you can use to order additional components.
iii. 500g of PLA for 3D printing. Additional material will come out of your $50 budget at 
5c/g.
Constraints: To add an element of freshness, this year rubber bands and other elastomeric
elements (e.g. rubber strips or strings) are banned (band?) and cannot form part of your 
robot design (see clause 1.6 of the fine print for details).
Technical support: Julian Murphy

<!-- page 3 -->
Classification: In-Confidence
The fine print:
1. Robots
1.1. Robots must be autonomous. There can be no human intervention in the robot’s operation, either 
physically or via software during the competition.
1.2. Robots must have a defined front end that will be used for aligning the start direction.
1.3. Robots must be controlled by the supplied Teensy 4.0 (ARM Cortex-M7 32-bit microcontroller). You 
cannot use other boards/controllers – unless you build one from discrete components (good luck 
with that!).
1.4. In addition to the Teensy and the tracked chassis components, you will be provided with:
1.4.1. A box of parts including various sensors, actuators, and structural elements such as aluminium 
plate, rods, acrylic etc.
1.4.2. A budget of $50 which you can use to order additional components. (see 5. Procurement).
1.4.3. 500g of PLA for 3D printing. Additional material will come out of your $50 (1.4.2) at 5c/g
1.5. All robots need to meet general safety standards, including:
1.5.1. Lasers must be below 5mW power, unless approved by Julian Phillips.
1.5.2. Spinning devices must travel at under 200rpm, unless adequate guarding is in place.
1.5.3. It is compulsory to use the supplied power module, in between the battery and any electronics.
1.5.4. No naked flames allowed; this includes any form of flame thrower.
1.5.5. No chemically explosive or EMP devices.
1.5.6. Voltage within your device should not exceed 100v DC.
1.6. Rubber bands and other similar elastomeric components with a high aspect ratio (e.g. strings or 
strips) cannot be used on any part of your robot
1.6.1. Elastomeric components are defined as polymers with high viscoelasticity allowing them to 
stretch significantly (>20% strain) under stress and return to their original shape when released.
1.6.2. High aspect ratio (flat length/width) > 5.0.
2. Environment
2.1. The competition will take place in a 2.4 x 4.9m meter arena with 400mm high walls. The walls and 
floor of the arena will be coloured black.
2.2. Within the arena will be two areas designated as home base.
2.2.1. These areas will be coloured blue and green.
2.2.2. The base area is defined as the 600 x 600mm coloured space. 
2.2.3. There will be a low ‘rim’ attached to the floor within each base (~10mm high).
2.2.4. The bases will be located in corners of the arena.
2.3. The arena will contain a number of obstacles that your robot will need to negotiate. Obstacles 
include walls, ramps and speed-bumps.
2.3.1. All gaps between walls (including arena walls) will be greater than 0.4m.
2.3.2. Vertical obstacles (e.g. walls and pipes) will be coloured red.
2.3.3. Horizontal obstacles (speed bumps, ramps) will be the same colour as the arena floor.
2.3.4. Speed-bumps will be rectangular in profile and up to 25mm high.
2.3.5. Ramps may be up to 100mm high, with a maximum gradient of 30%.
3. Weights and mobile elements (snitches and bludgers)
3.1. Within the arena, there will be more than 5 target weights.
3.2. All target weights will outwardly look identical.
3.2.1. Steel cylinder of diameter 50mm and height 70mm with an annular groove to facilitate 
gripping.
3.2.2. Target weights will weigh 1.0kg, 0.75kg, or 0.5kg.
3.3. Within the arena, there will be a number of dummy weights.
3.3.1. Dummy weights will be the same size and shape as target weights, but constructed of nonconducting plastic.
3.3.2. Some dummy weights will have a steel insert in the top.

<!-- page 4 -->
Classification: In-Confidence
3.4. Weights may be placed against arena or obstacle walls.
3.5. If a weight is knocked over during a round, it will be left on its side for the remainder of the round.
3.6. Within the arena there may be up to 2 mobile elements during the competition. Mobile elements
consist of Sphero Bolt robots (https://www.sphero.com/sphero-bolt)
3.6.1. Spherical plastic shell
3.6.2. Diameter of approximately 73mm
3.6.3. Speed of up to 7 m/s.
3.6.4. LED matrix may be lit up to distinguish between elements:
3.6.4.1. Snitches will be illuminated yellow (gold).
3.6.4.2. Bludgers will be illuminated red.
3.6.5. Path will be random. 
4. Competition
4.1. The competition consists of a number of rounds in which two robots are present in the arena 
simultaneously.
4.1.1. Robots will begin each round on their respective base, facing a direction specified by the 
competition director for each round.
4.1.2. The duration of each round is 2 minutes.
4.1.3. At the end of each round, the team will turn their robot off but will not lift it up. The robots 
will be picked up by ‘officials’ and taken to the judge’s desk for scoring – this is to prevent 
cheeky tilting of the robot to retain or drop weights. 
4.2. The robot with the highest score at the end of the round will be declared the winner.
4.2.1. The score for a robot is calculated at the end of the round by: Score = 1x (weight of target 
weights on board robot) + 2x (weight of target weights in home base at the end of the round) + 
3kg x (number of snitches on board robot) - 3kg x (number of bludgers on board robot) - 0.5kg x 
(number of dummy weights on board robot) - any penalties
4.3. A weight/mobile element will be considered on board a robot when it is off the ground and 
completely under the control of the robot i.e. if the robot is picked up off the ground, the 
weights/mobile element come with the robot. 
4.4. Weights on board a robot are not considered safe and can therefore be pilfered by the opposing 
team, provided that no deliberate damage is inflicted during the raid (as per 4.18).
4.5. Target weights that have been delivered to the team base are considered safe and cannot be stolen
by the opposing robot. 
4.5.1. A weight is considered delivered if it is on the ground touching any part of the coloured home 
base area, and not on board/under the control of a robot. 
4.5.2. A weight is considered stolen if it is brought on board the opposing robot (as per 4.3)
4.5.3. If one or more target weights is stolen from a base, those weights will still contribute to the 
final score of the team that originally gathered them and a penalty equivalent to this score will 
be subtracted from the thieving team. If it is unclear which weight(s) was stolen, it will be 
assumed to be the heaviest weight(s) among the target weights the thieving team has on board 
and in their home base. 
4.5.4. If one or more target weights are pushed outside the base without being stolen and are not 
returned by the end of the round, they will not count towards the score.
4.6. Dummy weights on board a robot subtract from the score (as per 4.2.1).
4.6.1. Dummy weights that have been delivered to a team base (and are therefore not on board a 
robot) do not contribute to the score.
4.7. A robot can have a maximum of 3 target weights on board at the end of a round.
4.7.1. Only 3 target weights can contribute to the on board part of the final score for the round. 
4.7.2. Dummy weights do not count towards the on board total. A robot can have many dummy 
weights on board – but they will count towards the final score.
4.7.3. If a robot ends the round with more than 3 target weights on board, a penalty will be applied.
4.7.3.1. For every additional target weight (>3) on board, a penalty of -1 points will be applied to 
the score at the end of the round.

<!-- page 5 -->
Classification: In-Confidence
4.7.3.2. Penalties will be limited so that the minimum score is 0 points.
4.7.3.3. If a robot finishes a round with >3 weights on board, the lightest target weights will 
contribute to the score, and the heaviest target weights will be omitted.
4.8. If two robots finish with the same score, the team with the greatest cumulative weight successfully 
delivered to their home base will be declared the winner.
4.9. If two robots finish with the same score and weight on their base, the team with the greatest 
number of target weights (on board and at base) will be declared the winner (via more successful 
pick-ups).
4.10. If two robots finish and are still tied according to 4.9, the team with the fewest number of dummy 
weights will be declared the winner.
4.11. In the unlikely event that two robots finish the round tied according to 4.10, the lightest robot will be 
declared the winner.
4.12. In the almost impossible event that two robots finish the round tied according to 4.11, the robot that 
has obviously travelled the greatest distance during the round (at the discretion of the competition 
director on the day) will be declared the winner. If one robot has not obviously travelled further than 
another, then the robot that is furthest from its base (in a straight line) will be declared the (dubious) 
winner.
4.13. In the ‘this is just getting ridiculous’ event that the round is tied according to 4.12, we’ll flip a coin…
4.14. There will be no restarts of robots. If a robot malfunctions or is damaged accidentally, it must 
continue to the end of the round without intervention.
4.14.1. If both teams malfunction at the start of a round, the round may restart at the discretion of 
the competition director on the day.
4.14.2. If both robots malfunction/get stuck during a round, the competition director may offer the 
opportunity to restart of reposition the robots to continue to the round. Both teams must agree 
for this to occur. 
4.15. Robots can be repaired between rounds. However, they must be ready in time for their next round. 
4.16. If a robot is late to its round, it will be disqualified. This decision will be made at the discretion of the 
competition director. There will be no discussion or appeal.
4.17. Both robots must remain inside the arena at all times. If a robot ventures outside the arena, 
whichever robot was responsible will be disqualified (i.e. if a robot takes itself outside the arena, it 
will be disqualified. If one robot is placed outside the arena by the opposition, the opposition will be 
disqualified). 
4.18.Any deliberate damage to the other robot during a knockout round will result in immediate 
disqualification. This is not Robot Wars! 
4.18.1. However, cunning methods to inhibit your competition that do not result in damage will be 
permitted. The decision on whether any damage is accidental or deliberate will be made by the 
competition director on the competition day – there will be no discussion or appeal, so be 
careful!
4.19. In the event that one robot is captured by another, any weights on board the captured robot will be 
considered property of the captor.
4.19.1. A robot is considered captured if no part of it is touching the ground and its movements are 
completely under the control of the captor – i.e. the captor must be able to move the captive 
robot at will and without restriction.
4.20.When the competition is completed, the design will be stripped down to the original components by 
the team and all parts returned to Julian – unless your robot is so damned awesome that you are 
requested to return the robot in one piece for a victory parade.
4.20.1. Robot strip down must be completed by the end of study week, else a penalty of 10% will be 
imposed on the team score for the final competition.
4.21. The competition will follow a double-elimination format, so a robot must lose 2 rounds before being 
eliminated.

<!-- page 6 -->
Classification: In-Confidence
5. Procurement
5.1. When spending your budget, items must be ordered through Julian and be able to be purchased 
using a credit card. If you buy any items yourself, or bring them from home, you will not be 
reimbursed. 
5.2. Any items not ordered through Julian will be assessed for value (either on production of a receipt, or 
Julian’s experience) and that amount deducted from your budget. This is not an all-out robot-beauty 
pageant, there are cost and hardware constraints, just like in the real-world.
5.2.1. Second-hand items will be valued at their current ‘new’ price (if you were to buy that item 
today)
5.3. Procurement requests should be submitted to Julian via email and procurement will take place 
fortnightly, unless agreed in advance.
5.4. All materials used from the supplied parts kit or additional procurement shall be identified in a bill of 
materials in the design document.
5.5. No budget credit will be given for unused items in the supplied parts kit.
Reporting: 
We have produced guidelines for each of the three reports that are required for this project. 
These guideline documents will be available on Learn. Basically, the structure of the three 
reports is: 
1. Conceptual design report – research, present and, evaluate several possible concepts 
and select one to take forward
2. Design progress report – Present progress to date on the design, including some 
technical aspects such as engineering drawings, testing/verification results, fault tree 
analysis.
3. Design evaluation report - critically evaluate the performance of your final design 
and compare it against competing designs in the context of this competition.
These reports have strict page limits to ensure that you present only relevant and important 
information. The requirements for these reports have changed slightly from previous years 
and updated documents will be made available soon. Several examples of each type from 
previous years are available on Learn – these will differ slightly from what you will need to 
produce but will provide some inspiration.
