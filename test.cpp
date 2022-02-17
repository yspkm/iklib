#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include "iklib.h"
#include <chrono>

using namespace std;
using namespace chrono;

#define PX_MAX 2
#define PX_MIN -2
#define PY_MAX 2
#define PY_MIN -2
#define PZ_MAX 2
#define PZ_MIN 0

#define ROLL_MAX PI
#define ROLL_MIN -PI
#define PITCH_MAX PI
#define PITCH_MIN -PI
#define YAW_MAX PI
#define YAW_MIN -PI

void print_vector3(char *name, v3_t *v)
{
    int i;
    printf("%-11s = [", name);
    printf("%16.8lf, ", v->v0);
    printf("%16.8lf, ", v->v1);
    printf("%16.8lf]\n", v->v2);
}

int main(void)
{
    ik_state_t ik_state;
    ik_input_t ik_input;
    char menu;
    int sign, rand_temp, i;
    type_t temp;

    system_clock::time_point start;
    system_clock::time_point end;
    nanoseconds nanosec;
    clock_t start_clk, end_clk;

    srand((unsigned)time(NULL));
    for (;;)
    {
        printf("0: test_values, 1:manual, 2: exit, others:rand with URDF > ");
        scanf("%c", &menu);
        getchar();
        if (menu == '0')
        {
            ik_input.target_pos.v0 = 0.49792;
            ik_input.target_pos.v1 = 1.3673;
            ik_input.target_pos.v2 = 2.4988;

            ik_input.mkdh_rpy.v0 = 0;
            ik_input.mkdh_rpy.v1 = -PI / 2;
            ik_input.mkdh_rpy.v2 = PI;

            ik_input.target_rpy.v0 = 0.366;
            ik_input.target_rpy.v1 = -0.078;
            ik_input.target_rpy.v2 = 2.561;
        }
        else if (menu == '1')
        {
            printf("target position (px py pz) : ");
            scanf("%lf %lf %lf", &ik_input.target_pos.v0, &ik_input.target_pos.v1, &ik_input.target_pos.v2);
            getchar();
            printf("target rpy (roll pitch yaw) : ");
            scanf("%lf %lf %lf", &ik_input.target_pos.v0, &ik_input.target_pos.v1, &ik_input.target_pos.v2);
            getchar();
            printf("1:URDF, others:others > ");
            menu = ' ';
            scanf("%c", &menu);
            getchar();
            if (menu == '1')
            {
                // URDF(ROS)->DH frame(Cur System)
                ik_input.mkdh_rpy.v0 = 0;
                ik_input.mkdh_rpy.v1 = -PI / 2;
                ik_input.mkdh_rpy.v2 = PI;
            }
            else
            {
                printf("mkdh_rpy (mkdh_roll mkdh_pitch mkdh_yaw) : ");
                scanf("%lf %lf %lf", &ik_input.mkdh_rpy.v0, &ik_input.mkdh_rpy.v1, &ik_input.mkdh_rpy.v2);
                getchar();
            }
        }
        else if (menu == '2')
        {
            printf("Exit\n");
            break;
        }
        else
        {
            ik_input.mkdh_rpy.v0 = 0;
            ik_input.mkdh_rpy.v1 = -PI / 2;
            ik_input.mkdh_rpy.v2 = PI;
            rand_temp = rand();
            temp = (rand_temp / (type_t)RAND_MAX) * (type_t)((PX_MAX - PX_MIN) / 2);
            rand_temp = rand();
            sign = pow(-1, rand_temp);
            ik_input.target_pos.v0 = (PX_MAX + PX_MIN) / 2.0 + temp * sign;

            rand_temp = rand();
            temp = (rand_temp / (type_t)RAND_MAX) * (type_t)((PY_MAX - PY_MIN) / 2);
            rand_temp = rand();
            sign = pow(-1, rand_temp);
            ik_input.target_pos.v1 = (PY_MAX + PY_MIN) / 2.0 + temp * sign;

            rand_temp = rand();
            temp = (rand_temp / (type_t)RAND_MAX) * (type_t)((PZ_MAX - PZ_MIN) / 2);
            rand_temp = rand();
            sign = pow(-1, rand_temp);
            ik_input.target_pos.v2 = (PZ_MAX + PZ_MIN) / 2.0 + temp * sign;

            rand_temp = rand();
            temp = (rand_temp / (type_t)RAND_MAX) * (type_t)((ROLL_MAX - ROLL_MIN) / 2);
            rand_temp = rand();
            sign = pow(-1, rand_temp);
            ik_input.target_rpy.v0 = (ROLL_MAX + ROLL_MIN) / 2.0 + temp * sign;

            rand_temp = rand();
            temp = (rand_temp / (type_t)RAND_MAX) * (type_t)((PITCH_MAX - PITCH_MIN) / 2);
            rand_temp = rand();
            sign = pow(-1, rand_temp);
            ik_input.target_rpy.v0 = (PITCH_MAX + PITCH_MIN) / 2.0 + temp * sign;

            rand_temp = rand();
            temp = (rand_temp / (type_t)RAND_MAX) * (type_t)((YAW_MAX - YAW_MIN) / 2);
            rand_temp = rand();
            sign = pow(-1, rand_temp);
            ik_input.target_rpy.v0 = (YAW_MAX + YAW_MIN) / 2.0 + temp * sign;
        }

        printf("\n<input>\n");
        print_vector3((char *)"target_pos", &ik_input.target_pos);
        print_vector3((char *)"target_rpy", &ik_input.target_rpy);
        print_vector3((char *)"mkdh_rpy", &ik_input.mkdh_rpy);

        init_ik_lib(&ik_state, &ik_input);

        printf("\n<output>\n");
        get_6joint_angles(&ik_state);
        printf("free var qi = [");
        for (i = 1; i < 6; i++)
        {
            printf("%16.8lf, ", get_q(i));
        }
        printf("%16.8lf]\n", get_q(i));

        printf("joint angle = [");
        for (i = 1; i < 6; i++)
        {
            printf("%16.8lf, ", get_theta(i));
        }
        printf("%16.8lf]\n", get_q(i));
        printf("\n");

        // for accurate measurement
        start = system_clock::now();
        get_6joint_angles(&ik_state);
        end = system_clock::now();

        // for accurate measurement
        start_clk = clock();
        get_6joint_angles(&ik_state);
        end_clk = clock();

        nanosec = end - start;
        printf("<elapsed time>\n%lf ms\n", nanosec.count() / 1000000.0);
        printf("%lu clocks\n",  end_clk - start_clk);
        printf("\n\n"); 
    }
}