<!-- Progress Report 1 (CDR draft) — markdown export. Contains inductive + ultrasound test data (section 5). -->
Group 23 – Anita Avia, Finn Buckley, Finlay Fairweather-Logie

ENMT301 Progress Report 1 – Robocup

## 1. Introduction 

*Max ½ Page, typical intro, outline general approaches, potential strategies (for all designs *

*proposed) *

This report documents the conceptual design process for the 2025 Robocup competition. Robocup is a competition between two purpose-built robots in a 2.4x4.9m arena containing obstacles. Robots must collect target weights while avoiding dummy weights, to score the most points at the end of the two minute round [1]. The report outlines research on possible features, presents a comprehensive list of requirements necessary for success in the competition and proposes three conceptual designs for a robot. Each design is then feature tested and critically evaluated using a decision matrix to select the strongest concept.

Performance in the competition largely comes down to how well the robot performs in three key categories: Navigation, Weight Identification, and Weight Collection. Additionally, a high-performing robot can also drop weights back at the ‘home base’ which enables a higher score to be achieved. All designs and requirements for the robot were structured around these categories, with additional constraints (such as ease of assembly and repair) considered to complement the main categories.

The overarching design principle used in concept generation is simplicity, as simple robots are easier to build, repair, and troubleshoot. Designs must use components provided by the competition, or acquired with the $50 supplementary budget, so complex systems likely will use elements not available to the team. Extensive research on old designs was undertaken to examine past approaches and select the most effective past features based on testing and results from previous competitions. Creativity is another key design principle, to explore novel solutions that achieve the design brief and have not been used in past years. The final design is selected using a decision matrix weighted against key categories and design principles, to identify the concept that will perform best in the competition. 

REMEMBER: Tables, Annotated images, captions etc do not count towards the word limit

## 2. Research 

*Max 2 pages (500 words), make observations about at LEAST 3 interesting features from past *

*designs. Include sketches, some quantified info (tables) proving effective/ineffectiveness of the *

*design. Don’t have to use in design, useful for informing designs. CHECK RULES for if features are viable for this year compared to last.*

Use concept summary table to save words. Make sure to have at least one figure per

### Pick up mechanism: (160 words)

Based on the robots from 2025 competition there were three common pick-up mechanisms. These were assessed in Table 1.

Table 1: 2025 Pick up mechanism comparison

| Metric | Ramp (Team 36) | Crane (Team 32) | Harvester (Team 12) |
| --- | --- | --- | --- |
| Average number of weights (metal or plastic) picked up per round | 2 | 1.67 | 3.14 |
| Average pick-up time | 2.25s | 4s, | 1.5s |
| % Picked up on first attempt | 77% | 83% | 73% |
| % Picked up on second attempt | 0% | 0% | 15% |
| % Weights missed due to pick up failure | 23% | 17% | 12% |
| Total snitches caught | 0 | 0 | 2 |
| Snitches encountered in an ideal position | 0 | 0 | 2 |
| Image |  |  |  |

Harvesters were the fastest and more robust option as they picked up the most snitches and weights in unique positions. Due to this year’s rubber-band ban this would need to be made of solid material, such as a rotating 3d printed drum. This adds complexity to design a robust and reliable harvester shape. Ramp designs were useful in directing the weights towards the robot and were used to some degree by all three teams. Team 32’s ramp design worked well when it encountered a weight however its small capturing area meant weights were missed often. The metallic crane was most reliable as it was able to pick up most weights in any orientation first try. However, this was slow as the robot needed to stop to pick up weights before it could continue. It also had no apparent method for snitch capture.

### Sort and store mechanism: 

One key category for robot performance is Weight Identification. Weights can be identified before collection or sorted onboard. One implementation from past years is using an inductive sensor onboard, so real metal weights trigger the sensor and are stored, while dummy weights do not and are discarded. This system was used frequently in past years, and table _ demonstrates that these groups performed at or better than average in weight identification, suggesting the system is effective.

Table 2: Inductive sensor past results

| Group, Year | Total Sorting Success | Year group average sorting success |
| --- | --- | --- |
| G27, 2025 | 87.5% (7 real, 1 dummy) | 84.4% (194 real, 38 dummies, 12 snitches) |
| G24, 2025 | 84.6% (10 real, 1 dummy, 1 snitch) | 84.4% (194 real, 38 dummies, 12 snitches) |
| G21, 2024 | 100% (6 real) | 94.5% (150 real, 9 dummies, 4 snitches) |
| G9, 2024 | 100% (6 real) | 94.5% (150 real, 9 dummies, 4 snitches) |

Examination of round footage revealed another advantage of this mechanism, as weights were dropped behind the robot, so the dummies did not trip the sensors again. The simplest storage implementation leaves the release gate open until the sensor triggers, closing it to retain metal weights as shown in Figure 1. However, this requires fast implementation of the gate, otherwise metal weights can also fall out the back. Another limitation of the mechanism is that the LJ18A3-8-Z/BY Inductive Sensor only has an 8mm range, requiring weights to pass in close proximity to the sensor.

Figure 1: Inductive Sensor Sorting System[^c1]

### Moving and navigation mechanism: 

Possibly the most important specification for robot performance is movement. This year there is a blanket ban on any elastomeric elements greatly reducing the possible options for chassis selection. Previously 3D printed wheels could appear as incredibly desirable mechanisms for movement, however due to O-rings being a key part of creating grip to the arena floor this design is much less practical comparatively to the provided track chassis which offers reliable grip across the arena surface and over obstacles such as the 25 mm speed bumps and ramps up to 100 mm high with 30% gradient. There are two key ways that the tracks can be implemented. By having the track teeth facing outward (contacting the ground), which provides more grip to the arena floor yet requires over tensioning the drive chain to hold the track in place effectively. Or by having the track teeth facing inward, mating them with a toothed driver wheel to hold the ruts in place ensuring that the tracks won’t slip along the drive chain, yet this option yields less grip on the arena floor. [^c2]

Navigation

## 3. Requirements 

1     Functional requirements: 

- The robot must have a means to differentiate between:

- Plastic, light dummy weights and heavy metal weights.

- Red and gold coloured Sphero’s.

- Robot should know its location, including both bases and the general arena.

- Once stored, weights should not be able to be stolen by other teams or lost due to a collision.

- Robot should detect looping after the third occasion, and change behaviour. *Rationale: Looping behaviour is a major waste of the limited round time.*

- Robot should return to base reliably and in 30 seconds from anywhere on the course. *Rationale: Weights stored at home base are worth more points.*

- Performance requirements:

- Sensors should detect weights 80% percent of the time.

- Robot should collect detected weights 90% of the time.

- The robot should be able to pick up weights in any orientation within 10 seconds of detection.

- Robot should operate with 70% of top speed when carrying a full payload of weights (2kg). 

- Non-functional requirements: 

- The robot must operate[^c4] for a minimum of 2 minutes. *Rationale: This is the length of a competition round. *

- The robot must be able to traverse the following without toppling, damaging, or trapping the robot:

- The robot must be able to traverse rectangular speed bumps 25mm high.

- The robots must be able to traverse ramps up 100mm high with a gradient of 30%.

- The robot must have a well thought-out and high-quality build, including:

- Clean wiring which is tightly secured and isn’t near moving parts. *Rationale: To ensure **electrical signals **are secure and** **wires are not damaged by** **the robot. *

- Durable and robust core features (movement, weight sensing and collection).

- The option to disassemble, replace any part, and reassemble in 30 hour. *Rationale: Rounds can be **in quick succession, and **the robot needs to have all parts working. *

- Constraints:  

- Robot must have a maximum length and width of 400mm. *Rationale: To ensure the robot can maneuver around all obstacles.*

- Robot must not attempt sabotage of competitors. Tinfoil balls? *Rationale: Research of sabotage has proved it is difficult and time-costly, so resources would be better spent elsewhere.*

- Operational requirements: 

- Robot should operate from 5 to 30 degrees Celsius. 

- Sensors should operate under normal lighting conditions and typical electrical noise conditions.

- Parts used should have a lifetime exceeding 50 uses. *Rationale: Parts should not fail from wear and tear during the competition**. *

## 4. Proposed Concepts

*Max 3 pages per concept, max 500 words per concept – this part is graded individually (worth *

*35% out of potential marks for the report – 65% is group) however expected that we help each *

*other as we are a team. Focused on **HARDWARE** design of the robot, **features concerning *

*specific behaviour required for competition strategy, present hardware concepts** preferentially *

*using sketches, tables, diagrams etc. Convincing marker of credibility **&** feasibility of concept *

*via testing of important components + features. Testing may enable objective comparison*

*between designs. *

### Start of AAs concept:

 

## 5. Preliminary Results

*Max 2 pages, 500 words, at least 3 tests + data to validate and/or compare features of *

*developed concepts. Ideally this section will be figures/photos/tables with brief *

*explanation/justification. Include evaluation of why tests were chosen, what impact the finding *

*will have on the chosen concept. Example tests include – testing electromagnet strength (including diff orientations + current draw), range of inductive prox sensor, testing force required to push weights up different *

*inclined ramps.* 

Things to test/ measure:

- size of weights/spheros

- how good colour detector is

- how well the inductive sensor works

If combine harvester works

Inductive sensor testing:

| Position | Maximum distance to sense metallic weight |
| --- | --- |
| Bottom: fully metallic | 7mm |
| Top: fully metallic | 7mm |
| Held on an angle: fully metallic | 6mm |
| Bottom: plastic | NA |
| Top: plastic | NA |
| Held on an angle: plastic | NA |
| Plastic with metal insert: top | 7mm |
| Plastic with metal insert: side | NA |
| Plastic with metal insert: side on an angle | 1mm |

Ultrasound testing:

| Distance from object to sensor | Value read |
| --- | --- |
| 0mm | 814 |
| 5mm | 3 |
| 10mm | 2mm (&7) |
| 20mm | 2 (&1) |
| 30mm | 3 |
| 40mm | 3 |
| 50mm | 4&5 |
| 75mm | 7 |
| 100mm | 10 (&9) |
| 105mm | 11 |
| 200mm | 20 |
| 500mm | 50 (+-5) |
|  |  |
|  |  |
|  |  |
|  |  |

*(include comment about deadband, 0-1mm reads 814)

(therefore given in cm)

Pretty accurate but 1cm tolerance – factor in poor measruing for greater range of values read

After 500mm, +-5cm error, after 700mm unreliable 

## 6. Concept Evaluation 

*Max 2 pages, 500 words, evaluate/compare the concepts using a decision matric. Doesn’t need *

*to be super detailed/accurate, just enough to evaluate + compare concepts in a relatively *

*objective manner relative to the stated design requirements. Justify rating system in a clear and *

*concise manner, strongly encourages to use the AHP process for assigning weights to criteria. *

*Briefly discuss/comment on the concepts in the context of the competition and our *

*requirements. *

## 7. Conclusion & Recommendations 

*Max ½ page, based on evaluations, make a recommendation for one to be developed, back it up *

*by summarising benefits/why. Doesn’t have to be the one we build but should be.* 

Can also combine features from multiple concept designs if they are good

## 8. References

[1]   C. Pretty, 2026 Robocup challenge, University of Canterbury, 2025

## 9. Appendix 

(Including group charter)

## Comments

[^c1] Finn Buckley: Choose figure
[^c2] Finn Buckley: INSERT stat on top 5 teams using tracks/not and ruts in/outside could be formed as table.
[^c4] Anita Avia: Probs need a more specific definition than this