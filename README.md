
# Sessions

- ~~Thu 2: Introductory Lecture~~
- ~~Thu 2: Project Management~~
- ~~Mon 6: Teamwork Lecture~~
- ~~Tue 7: Progress Meeting~~
- ~~Wed 8: Session~~
- Thu 9: Session
- Mon 13: Session
- Tue 14: Progress Meeting + **PEER ASSESMENT**
- Wed 15: Session
- Thu 16: Session
- Mon 20: Session
- Tue 21: Progress Meeting
- Wed 22: Session
- Thu 23: **IDP COMPETITION**

# Links
- Google Drive: https://drive.google.com/drive/folders/1ARmU0Phr4KM0iht5VVzTSHMOpqvWqKKI?usp=sharing
- Moodle: https://www.vle.cam.ac.uk/course/view.php?id=163282
- Spotify: https://open.spotify.com/playlist/7319K8yNM7Xe6g0ajQ0TMM?si=jNb81WRST5qzq9ByQZrZsg&pt=36b66da757990dbd584ce3efc22457c7

# Values for motors

| Description | Symbol | Value |
| --- | --- | --- |
| Radius of the wheels | $r_W$ | $3.5cm$ |
| Distance between the wheels (width of axle) | $D_W$ | $23cm$ |
| Distance from the line sensor to the axle | $a_L$ | $??$ |
| Distance between line sensors on the axle | $a_A$ | $??$ |
| Motor frequency | $f_M$ | $? RPM = ? Hz$ |
| Velocity when moving forward | $u$ | $2\pi f_M r_W$ |
| Angular velocity when rotating | $\omega$ | $2 \pi f_M r_W / D_W$ |
| Delay needed per unit distance moved | $1/u$ | $? ms/m$ |
| Delay needed per degree rotated | $1/\omega$ | $? ms/\degree$ |

# First Algorithm

Is this correct?

We number the points. Point 1 is at the inital position. Point 2 is where the car will first reach the white line when moving forwoard from ttghe initiasl position. 
From there points are numbered from number 3 to 12 in a clockwise fashion. 

1. We exit going forward until we reach the white line. (P1 -> P2)
2. We follow the line until P3. (P2 -> P3)
3. We check every point in a clockwise order:
    - 3.1. We use the line following algorithm to keep the robot on the line.
    - 3.2. When the 2 sideways sensors detect a junction, we check if it matches the correct junction at that point. If we need to rotate (for example if we are at P3, P5 or P6), we rotate first.
4. When we detect a cube, we light up the LEDs based on the magnetic sensor. 
5. We push it until the axle reaches the junction point. We rotate by 180 deg.
5. We go down the same path with the cube in front. We use the same algorithm as in steps 1-3. (CUBE->P1)
6. Once we reach point 1, we rotate by +- 90 deg, depending on the color of the block. We move forward by the distance between the two stations.
7. We go back to the initial station. We rotate to the initial state.
8. We repeat this whole process again.

# Ideas
- ???

