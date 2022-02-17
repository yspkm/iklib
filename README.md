# **6DOF robotic arm Inverse Kinematics Library with Modified Denavit–Hartenberg parameters**

## **1.  Relationship with IK-FPGA repository.**

I am implementing the inverse kinematic algorithm on FPGA. By the way, most of the open-source robotic libraries are too heavy, so it is hard to port into Verilog. My purpose was to calculate 6-free variables from 6-input variables and 28-DH parameters (25+3 dummy) and 3-frame-change parameters to change into DH-frame with all of them adjustable. As a by-product of seeking algorithms for Verilog, I made a lightweight and efficient 6dof robotic arm inverse kinematics library. I found that it takes less than two clock_t units to calculate 6-free variables (on average, returned from clock() method of time library), while the papers I've read have claimed that tens to hundreds of microseconds are fast. So, I think it isn't wrong to use it in embedded robotic arms that cannot use a network or ROS.

## **2. Theoretical background [1]**

### **2.1 q1**

![pic_0](https://user-images.githubusercontent.com/71680670/154224837-26fba191-d541-408a-a6ef-5ce477243194.PNG)

### **2.2 q2, q3: It corresponds to the planar positioning Case 4.**

![pic_1](https://user-images.githubusercontent.com/71680670/154224873-22f825c1-1129-4035-b1e7-269f19716ac9.PNG)

![pic_2](https://user-images.githubusercontent.com/71680670/154224886-84e18fab-1666-46df-b53d-4a028fd330a8.PNG)

### **2.3 q4, q5, q6**

$$
[0/G_{nonDH}, [-roll, -pitch, -yaw]]
$$

$$
[G_{DH/non_DH} : [-roll_{mkdh}, -pitch_{mkdh}, -yaw_{mkdh} ]]
$$

$$
R== Rx(roll)*Ry(pitch)*Rz(yaw)
$$

$$
\begin{aligned}
inv(R_{mkdh} * R_{wrist}) &== inv(R_{wrist}) * inv(R_{mkdh})\\
&== inv(Rx(-roll)*Ry(-pitch)*Rz(-yaw)) *inv(Rx(-roll_{mkdh})*Ry(-pitch_{mkdh})*Rz(-yaw_{mkdh}))\\
&== inv(Rz(-yaw))*inv(Ry(-pitch))*inv(Rx(-roll))*inv(Rz(-yaw_{mkdh}))*inv(Ry(-pitch_{mkdh}))*inv(Rx(-roll_{mkdh}))\\
&== Rz(yaw)) *Ry(pitch) * Rx(roll) * Rz(yaw_{mkdh}) * Ry(pitch_{mkdh}) * Rx(roll_{mkdh})
\end{aligned}
$$

$$
\begin{aligned}
R0G &== R03 * R36 * R6G\\
&== R03 * R36 * eye(3)
\end{aligned}
$$

$$
\begin{aligned}
transpose(R03) * R0G &== trnaspose(R03) * R03 * R36\\
&== eye(3) * R36  \ \ /* orthogonal \ \ transformation */
\end{aligned}
$$

$$
R36 = transpose(R03) * R0G
$$

The time for obtaining the values of 6 free variables takes less than two clock_t units with clock() function on average.
For tests, you can use the test code.

```shell
$ g++ -o test test.cpp iklib.c -lm
$ ./test
0: test_values, 1:manual, 2: exit, others:rand with URDF > 0

<input>
target_pos  = [      0.49792000,       1.36730000,       2.49880000]
target_rpy  = [      0.36600000,      -0.07800000,       2.56100000]
mkdh_rpy    = [      0.00000000,      -1.57079633,       3.14159265]

<output>
free var qi = [      1.01249809,      -0.27580036,      -0.11568665,       1.63446527,       1.52050003,      -0.81578131]
joint angle = [      1.01249809,      -1.84659669,      -0.11568665,       1.63446527,       1.52050003,      -0.81578131]

<elapsed time>
0.001498 ms
1 clocks


0: test_values, 1:manual, 2: exit, others:rand with URDF > 1
target position (px py pz) : 1 -1 0.5
target rpy (roll pitch yaw) : -0.2 1 -0.3
1:URDF, others:others > 1

<input>
target_pos  = [     -0.20000000,       1.00000000,      -0.30000000]
target_rpy  = [      0.36600000,      -0.07800000,       2.56100000]
mkdh_rpy    = [      0.00000000,      -1.57079633,       3.14159265]

<output>
free var qi = [      1.50785765,       1.38507265,       0.66723941,       2.00444650,       1.87290832,       3.07245285]
joint angle = [      1.50785765,      -0.18572368,       0.66723941,       2.00444650,       1.87290832,       3.07245285]

<elapsed time>
0.001492 ms
1 clocks


0: test_values, 1:manual, 2: exit, others:rand with URDF > 3

<input>
target_pos  = [      1.08733312,      -0.15999474,       0.52172012]
target_rpy  = [     -1.80611864,      -0.07800000,       2.56100000]
mkdh_rpy    = [      0.00000000,      -1.57079633,       3.14159265]

<output>
free var qi = [     -0.23844455,       0.40383783,       0.76456971,       0.38157521,       2.02583087,       1.48227711]
joint angle = [     -0.23844455,      -1.16695849,       0.76456971,       0.38157521,       2.02583087,       1.48227711]

<elapsed time>
0.011210 ms
1 clocks


0: test_values, 1:manual, 2: exit, others:rand with URDF > 2
Exit
```

## **3. 메소드 및 구조체 설명**

1. 구조체
2. 외부 사용 함수
3. 내부 사용 함수
4. 테스트용 함수
5. 적용예시

##  **References**

**[1]	K. S. Fu, R. C. Gonzalez, and C. S. G. Lee, Robotics: Control, Sensing, Vision, and Intelligence. MCGRAW-HILL EDUCATION, 1987.**

**[2]**