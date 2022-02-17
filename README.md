# **6DOF Inverse Kinematics Library with Modified Denavit–Hartenberg parameters**

## **1.  Relationship with IK-FPGA repository.**

I have planned to start implementing the inverse kinematic algorithm on my Artix-7 100T FPGA in the summer of 2022 as a preliminary study of robotics classes that I am interested in and want to take lectures in the second semester. A lightweight algorithm that firstly controls a 6dof robotic arm could be found as open-source to achieve this final goal. Still, most of them are too heavy and complex with abstraction than my needs (to calculate 6-free variables from 6-input variables and 28-DH parameters (25+3 dummy) and 3-frame-change parameters to change into DH-frame with all of them adjustable). As a result, I should have to design it from underwater (2022-02-10 ~ 2022-02-14). Unexpectedly, As a by-product of seeking algorithms for Verilog, I made a lightweight and efficient 6dof robotic arm inverse kinematics library. I found that it takes only less than two clock_t units to calculate 6-free variables (on average, returned from clock() method of time library). More precisely, from chrono library, it is 0.000586 ms (=586.218669 ns) on average. It is pretty fast; therefore, I decided to create a new repository for this code.

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
For more accurate measurements, we can run the following code including chrono library.

```c++
    unsigned long long sum = 0;
    int loop = 1000000;
    system_clock::time_point start;
    system_clock::time_point end;
   for (int i = 0; i < loop; i++)
    {
        start = system_clock::now();
        get_6joint_angles(ik_state_ptr);
        end = system_clock::now();
        nanoseconds nanosec = end - start;
        sum += nanosec.count();
    }
    printf("avg ns: %lf\n", (double)sum / loop);
```

```shell
> avg ns: 586.218669
```

## **3. 메소드 및 구조체 설명**

1. 구조체
2. 외부 사용 함수
3. 내부 사용 함수
4. 테스트용 함수
5. 적용예시

##  **References**

**[1]	K. S. Fu, R. C. Gonzlez, and C. S. G. Lee, Robotics: Control, Sensing, Vision, and Intelligence. MCGRAW-HILL EDUCATION, 1987.**

**[2]**