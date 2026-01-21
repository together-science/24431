This is the MattAutoRemaster.md
For the purpose of documenting and optimizing the process of fixing the malfunctioning code

Problem A:

Strafe doesn't work

We have been performing tests with outdated code
We will now restart the process (regretably)

We will take advantage of this setback to make more standardized tests

We have created the autonomus AutoRemasterTest.java
In the control hub it is title ART v1.0.0
Version name in hub will change acordingly

---
Test #1:
    Relavent Notes or Comments
        Due to using the pratice chassis not all actions are appearant (cannon shooting)
    Environrment
        - Starts on point 0, 5 in respect to feet (position at the back center of the robot)
        - Explicitly initalized when already in position
        - Provided 5 - 10 seconds after initalization before start
    Observation
        - Moved forward
        - Scanned for april tag (camera and tag not present/unfunctional)
        - Turned towards goal
        - Waited 2-3 seconds (way too long)
        - Moved towards preferred ending position
        - Made two rotations then strafed left towards end of field
        - Timer expired and auto ended
    Conclusion
        - Unsatisfacory
        - Possible missuse of trigonometric functions
        - Will have to analyze programing
---

Goal vs Result
    Cannons spin up - No cannons on pratice chassis
    April tag observed - No functioning camera present
    Moves forward 78 inches - Does goal, however ocasionaly 