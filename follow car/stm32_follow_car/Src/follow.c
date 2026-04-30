/*
 * follow.c - 跟随控制与避障决策
 *
 * 工作逻辑 (优先级从高到低):
 * 1. 急停按钮断开 -> 立即停车 (最高优先级)
 * 2. 蓝牙手动控制激活 -> 执行手动方向指令 (第二优先级)
 * 3. 模式开关未在跟随模式 -> 停车
 * 4. UWB测量距离 >= 设定跟随距离 -> 启动跟随(前进)
 * 5. UWB测量距离 < 设定跟随距离(含迟滞) -> 停车
 * 6. 前进过程中:
 *    6a. 前方距离 < 紧急停车距离 -> 紧急刹停, 障碍解除后自动恢复
 *    6b. 正常避障: 差速转弯绕行
 *
 * 近距离防抖:
 *    - 距离迟滞: 进入跟随 >= follow_dist, 退出跟随 < follow_dist - 0.2m
 *    - 最低输出速度: PID输出>0时钳位到 MIN_EFFECTIVE_PWM
 *    - 近距离限制角度修正比例: speed低时 diff 不超过 speed*0.3
 *
 * 转向灯: PD0=左转, PD1=右转, 转弯时自动闪烁
 */
#include <stdlib.h>
#include "follow.h"
#include "motor.h"
#include "uwb.h"
#include "ultrasonic.h"
#include "lidar.h"
#include "oled.h"
#include "bsp.h"
#include "ble.h"

FollowState_t follow_state = STATE_IDLE;

/* 避障持续时间计数 */
static uint32_t avoid_start_tick = 0;
#define AVOID_MIN_DURATION_MS  300

/* 距离迟滞量 (米) */
#define DIST_HYSTERESIS_M  0.2f

/* 电机最低有效PWM (低于此值轮子不转只嗡嗡响) */
#define MIN_EFFECTIVE_PWM  120

/* 近距离角度修正限制比例 */
#define ANGLE_DIFF_MAX_RATIO  0.3f

/* ========== PID 控制器 ========== */
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float out_min, out_max;
    float integral_max;
} PID_t;

static PID_t pid_dist;
static PID_t pid_angle;

static void PID_Init(PID_t *pid, float kp, float ki, float kd,
                      float out_min, float out_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min    = out_min;
    pid->out_max    = out_max;
    pid->integral_max = out_max * 0.5f;
}

static void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

static float PID_Compute(PID_t *pid, float error, float dt)
{
    float p_term = pid->Kp * error;

    pid->integral += error * dt;
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float i_term = pid->Ki * pid->integral;

    float d_term = 0.0f;
    if (dt > 0.001f) {
        d_term = pid->Kd * (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;

    float output = p_term + i_term + d_term;
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;
    return output;
}

/* ========== 电机输出平滑 (斜坡限速) ========== */
static int16_t smooth_left  = 0;
static int16_t smooth_right = 0;
#define RAMP_STEP  30

static int16_t ramp_to(int16_t current, int16_t target)
{
    int16_t diff = target - current;
    if (diff > RAMP_STEP)       return current + RAMP_STEP;
    else if (diff < -RAMP_STEP) return current - RAMP_STEP;
    else                        return target;
}

static void Motor_SmoothDiff(int16_t left_target, int16_t right_target)
{
    smooth_left  = ramp_to(smooth_left,  left_target);
    smooth_right = ramp_to(smooth_right, right_target);
    Motor_ForwardDiff(smooth_left, smooth_right);
}

static void Motor_SmoothStop(void)
{
    smooth_left  = ramp_to(smooth_left,  0);
    smooth_right = ramp_to(smooth_right, 0);
    if (smooth_left == 0 && smooth_right == 0) {
        Motor_Stop();
    } else {
        Motor_ForwardDiff(smooth_left, smooth_right);
    }
}

/* ========== 初始化 ========== */
void Follow_Init(void)
{
    follow_state = STATE_IDLE;
    Motor_Stop();
    smooth_left  = 0;
    smooth_right = 0;

    PID_Init(&pid_dist,
             DEFAULT_PID_DIST_KP, DEFAULT_PID_DIST_KI, DEFAULT_PID_DIST_KD,
             0.0f, 666.0f);

    PID_Init(&pid_angle,
             DEFAULT_PID_ANGLE_KP, DEFAULT_PID_ANGLE_KI, DEFAULT_PID_ANGLE_KD,
             -666.0f, 666.0f);
}

/* ========== 传感器辅助函数 ========== */
static uint16_t get_front_us_min(void)
{
    uint16_t fl = us_data.valid[US_FRONT_LEFT]  ? us_data.distance_cm[US_FRONT_LEFT]  : 0xFFFF;
    uint16_t fr = us_data.valid[US_FRONT_RIGHT] ? us_data.distance_cm[US_FRONT_RIGHT] : 0xFFFF;
    return (fl < fr) ? fl : fr;
}

static uint16_t get_front_obstacle_cm(void)
{
    uint16_t us_front = get_front_us_min();
    uint16_t lidar_cm = lidar_data.valid ? (lidar_data.front_min_dist_mm / 10) : 0xFFFF;
    return (us_front < lidar_cm) ? us_front : lidar_cm;
}

static uint16_t get_left_dist(void)
{
    return us_data.valid[US_LEFT] ? us_data.distance_cm[US_LEFT] : 0xFFFF;
}

static uint16_t get_right_dist(void)
{
    return us_data.valid[US_RIGHT] ? us_data.distance_cm[US_RIGHT] : 0xFFFF;
}

/* ========== 蓝牙摇杆手动控制处理 ========== */
static void handle_ble_manual(void)
{
    int16_t joy_x = (int16_t)g_ble_joy_x;
    int16_t joy_y = (int16_t)g_ble_joy_y;

    follow_state = STATE_BLE_MANUAL;
    LED_AllOff();
    LED_Blue(true);
    LED_Yellow(true);

    if (joy_x > -8 && joy_x < 8) joy_x = 0;
    if (joy_y > -8 && joy_y < 8) joy_y = 0;

    if (joy_x < -15)      TurnSignal_Set(TURN_LEFT);
    else if (joy_x > 15)  TurnSignal_Set(TURN_RIGHT);
    else                   TurnSignal_Set(TURN_NONE);

    if (joy_x == 0 && joy_y == 0) {
        Motor_SmoothStop();
        OLED_ShowString(0, 6, "BLE STOP    ");
        return;
    }

    int16_t left_mix  = joy_y + joy_x;
    int16_t right_mix = joy_y - joy_x;

    int16_t max_val = abs(left_mix);
    if (abs(right_mix) > max_val) max_val = abs(right_mix);
    if (max_val > 100) {
        left_mix  = (int16_t)((int32_t)left_mix  * 100 / max_val);
        right_mix = (int16_t)((int32_t)right_mix * 100 / max_val);
    }

    uint16_t base_max = (uint16_t)g_car_params.motor_base_speed;
    if (base_max > 666) base_max = 666;

    MotorDir_t left_dir  = (left_mix >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
    MotorDir_t right_dir = (right_mix >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
    uint16_t left_spd  = (uint16_t)((uint32_t)abs(left_mix)  * base_max / 100);
    uint16_t right_spd = (uint16_t)((uint32_t)abs(right_mix) * base_max / 100);

    if (left_spd > 666)  left_spd = 666;
    if (right_spd > 666) right_spd = 666;

    Motor_SetLeft(left_dir, left_spd);
    Motor_SetRight(right_dir, right_spd);
    OLED_ShowString(0, 6, "BLE JOYSTICK");
}

/* ========== 主决策函数 ========== */
void Follow_Update(void)
{
    uint32_t now = HAL_GetTick();
    static uint32_t last_tick = 0;
    static float angle_error_filt = 0.0f;
    static bool is_following = false;  /* 迟滞状态: 当前是否在跟随中 */

    float dt = (float)(now - last_tick) / 1000.0f;
    if (dt > 0.5f) dt = 0.02f;
    last_tick = now;

    /* 读取动态参数 */
    float    follow_dist  = g_car_params.follow_distance_m;
    uint16_t obs_dist_cm  = (uint16_t)g_car_params.obstacle_dist_cm;
    uint16_t estop_cm     = (uint16_t)g_car_params.emergency_stop_dist_cm;
    uint16_t turn_spd     = (uint16_t)g_car_params.motor_turn_speed;
    uint16_t slow_spd     = (uint16_t)g_car_params.motor_slow_speed;
    uint16_t max_follow_spd = (uint16_t)g_car_params.max_follow_speed;
    if (max_follow_spd > 666) max_follow_spd = 666;

    /* 实时更新PID增益 */
    pid_dist.Kp  = g_car_params.pid_dist_kp;
    pid_dist.Ki  = g_car_params.pid_dist_ki;
    pid_dist.Kd  = g_car_params.pid_dist_kd;
    pid_dist.out_max = (float)max_follow_spd;
    pid_angle.Kp = g_car_params.pid_angle_kp;
    pid_angle.Ki = g_car_params.pid_angle_ki;
    pid_angle.Kd = g_car_params.pid_angle_kd;

    /* 当前UWB角度 (用于OLED显示) */
    int16_t cur_angle = uwb_data.valid ? uwb_data.angle_deg : 0;

    /* ===== 最高优先级: 急停 ===== */
    if (IsEstopPressed()) {
        follow_state = STATE_ESTOP;
        Motor_Stop();
        smooth_left = 0; smooth_right = 0;
        LED_AllOff();
        LED_Red(true);
        TurnSignal_Set(TURN_NONE);
        PID_Reset(&pid_dist);
        PID_Reset(&pid_angle);
        is_following = false;
        return;
    }

    /* ===== 第二优先级: 蓝牙手动控制 ===== */
    if (g_ble_manual_active) {
        PID_Reset(&pid_dist);
        PID_Reset(&pid_angle);
        is_following = false;
        handle_ble_manual();
        return;
    }

    /* ===== 检查模式开关 ===== */
    if (!IsFollowMode()) {
        follow_state = STATE_IDLE;
        Motor_SmoothStop();
        LED_AllOff();
        LED_Yellow(true);
        TurnSignal_Set(TURN_NONE);
        PID_Reset(&pid_dist);
        PID_Reset(&pid_angle);
        is_following = false;
        return;
    }

    /* ===== 跟随模式工作 ===== */
    LED_Yellow(false);

    /* 检查UWB数据 */
    if (!uwb_data.valid) {
        Motor_SmoothStop();
        LED_Blue(true);
        LED_Green(false);
        TurnSignal_Set(TURN_NONE);
        PID_Reset(&pid_dist);
        PID_Reset(&pid_angle);
        is_following = false;

        uint16_t front_obs = get_front_obstacle_cm();
        OLED_Update(0.0f, 0, front_obs,
                    lidar_data.valid ? lidar_data.front_min_dist_mm : 0xFFFF);
        OLED_ShowString(0, 6, "NO UWB DATA ");
        return;
    }

    float dist = uwb_data.distance_m;

    /* ===== 距离判断 (带迟滞) =====
     * 进入跟随条件: dist >= follow_dist
     * 退出跟随条件: dist < follow_dist - DIST_HYSTERESIS_M
     * 在迟滞区间内保持当前状态, 不反复切换 */
    float stop_threshold = follow_dist - DIST_HYSTERESIS_M;
    if (stop_threshold < 0.1f) stop_threshold = 0.1f;

    if (!is_following) {
        /* 当前停着, 需要距离 >= follow_dist 才启动 */
        if (dist < follow_dist) {
            follow_state = STATE_IDLE;
            Motor_SmoothStop();
            LED_Green(true);
            LED_Blue(false);
            TurnSignal_Set(TURN_NONE);
            /* 不 Reset PID, 避免频繁清零 */

            uint16_t front_obs = get_front_obstacle_cm();
            OLED_Update(dist, cur_angle, front_obs,
                        lidar_data.valid ? lidar_data.front_min_dist_mm : 0xFFFF);
            OLED_ShowString(0, 6, "IN RANGE    ");
            return;
        }
        /* dist >= follow_dist, 启动跟随 */
        is_following = true;
    } else {
        /* 当前在跟随中, 需要距离 < stop_threshold 才停 */
        if (dist < stop_threshold) {
            is_following = false;
            follow_state = STATE_IDLE;
            Motor_SmoothStop();
            LED_Green(true);
            LED_Blue(false);
            TurnSignal_Set(TURN_NONE);

            uint16_t front_obs = get_front_obstacle_cm();
            OLED_Update(dist, cur_angle, front_obs,
                        lidar_data.valid ? lidar_data.front_min_dist_mm : 0xFFFF);
            OLED_ShowString(0, 6, "IN RANGE    ");
            return;
        }
    }

    /* ===== 需要跟随前进 ===== */
    LED_Blue(true);
    LED_Green(false);

    uint16_t front_obs = get_front_obstacle_cm();
    uint16_t left_obs  = get_left_dist();
    uint16_t right_obs = get_right_dist();

    /* 更新OLED */
    static uint32_t oled_last = 0;
    if (now - oled_last > 200) {
        oled_last = now;
        OLED_Update(dist, cur_angle, front_obs,
                    lidar_data.valid ? lidar_data.front_min_dist_mm : 0xFFFF);
    }

    /* ===== 紧急停车 ===== */
    if (front_obs < estop_cm) {
        follow_state = STATE_EMERGENCY_STOP;
        Motor_Stop();
        smooth_left = 0; smooth_right = 0;
        LED_Red(true);
        LED_Blue(false);
        TurnSignal_Set(TURN_NONE);
        PID_Reset(&pid_dist);
        PID_Reset(&pid_angle);
        OLED_ShowString(0, 6, "!! E-STOP !!");
        return;
    }

    /* 从紧急停车恢复 */
    if (follow_state == STATE_EMERGENCY_STOP) {
        LED_Red(false);
        follow_state = STATE_FOLLOW;
    }

    /* ===== 避障逻辑 ===== */
    bool front_blocked = (front_obs < obs_dist_cm);
    bool left_blocked  = (left_obs  < obs_dist_cm);
    bool right_blocked = (right_obs < obs_dist_cm);

    /* 避障最小持续时间 */
    if ((follow_state == STATE_AVOID_LEFT || follow_state == STATE_AVOID_RIGHT) &&
        (now - avoid_start_tick < AVOID_MIN_DURATION_MS)) {
        if (follow_state == STATE_AVOID_LEFT) {
            Motor_TurnLeft(turn_spd, slow_spd);
            TurnSignal_Set(TURN_LEFT);
            OLED_ShowString(0, 6, "AVOID LEFT  ");
        } else {
            Motor_TurnRight(turn_spd, slow_spd);
            TurnSignal_Set(TURN_RIGHT);
            OLED_ShowString(0, 6, "AVOID RIGHT ");
        }
        PID_Reset(&pid_dist);
        PID_Reset(&pid_angle);
        return;
    }

    if (front_blocked) {
        PID_Reset(&pid_dist);
        PID_Reset(&pid_angle);

        if (!left_blocked && !right_blocked) {
            if (left_obs >= right_obs) {
                follow_state = STATE_AVOID_LEFT;
                Motor_TurnLeft(turn_spd, slow_spd);
                TurnSignal_Set(TURN_LEFT);
            } else {
                follow_state = STATE_AVOID_RIGHT;
                Motor_TurnRight(turn_spd, slow_spd);
                TurnSignal_Set(TURN_RIGHT);
            }
        } else if (!left_blocked) {
            follow_state = STATE_AVOID_LEFT;
            Motor_TurnLeft(turn_spd, slow_spd);
            TurnSignal_Set(TURN_LEFT);
        } else if (!right_blocked) {
            follow_state = STATE_AVOID_RIGHT;
            Motor_TurnRight(turn_spd, slow_spd);
            TurnSignal_Set(TURN_RIGHT);
        } else {
            follow_state = STATE_IDLE;
            Motor_Stop();
            smooth_left = 0; smooth_right = 0;
            LED_Red(true);
            TurnSignal_Set(TURN_NONE);
            OLED_ShowString(0, 6, "BLOCKED STOP");
            return;
        }
        avoid_start_tick = now;
        OLED_ShowString(0, 6, follow_state == STATE_AVOID_LEFT ?
                        "AVOID LEFT  " : "AVOID RIGHT ");

    } else if (left_blocked && !right_blocked) {
        follow_state = STATE_AVOID_RIGHT;
        uint16_t base_spd = (uint16_t)g_car_params.motor_base_speed;
        Motor_TurnRight(base_spd, base_spd / 2);
        avoid_start_tick = now;
        TurnSignal_Set(TURN_RIGHT);
        PID_Reset(&pid_dist);
        PID_Reset(&pid_angle);
        OLED_ShowString(0, 6, "NUDGE RIGHT ");

    } else if (right_blocked && !left_blocked) {
        follow_state = STATE_AVOID_LEFT;
        uint16_t base_spd = (uint16_t)g_car_params.motor_base_speed;
        Motor_TurnLeft(base_spd, base_spd / 2);
        avoid_start_tick = now;
        TurnSignal_Set(TURN_LEFT);
        PID_Reset(&pid_dist);
        PID_Reset(&pid_angle);
        OLED_ShowString(0, 6, "NUDGE LEFT  ");

    } else {
        /* ===== 无障碍物, PID跟随 ===== */
        follow_state = STATE_FOLLOW;

        /* 距离PID */
        float dist_error = dist - follow_dist;
        float speed_f = PID_Compute(&pid_dist, dist_error, dt);
        int16_t speed = (int16_t)speed_f;

        /* 方案2: 最低有效速度保底 — PID有输出时不低于电机启动阈值 */
        if (speed > 0 && speed < MIN_EFFECTIVE_PWM) {
            speed = MIN_EFFECTIVE_PWM;
        }

        /* 角度误差低通滤波 (alpha=0.08, 强滤波抑制UWB抖动) */
        float raw_angle_error = (float)uwb_data.angle_deg;
        const float angle_alpha = 0.08f;
        angle_error_filt += angle_alpha * (raw_angle_error - angle_error_filt);
        float angle_error = angle_error_filt;

        /* 死区 */
        float tolerance = g_car_params.uwb_angle_tolerance_deg;
        if (angle_error > -tolerance && angle_error < tolerance) {
            angle_error = 0.0f;
        }

        /* 角度PID */
        float diff_f = PID_Compute(&pid_angle, angle_error, dt);

        /* 方案3: 近距离限制角度修正幅度 — diff不超过speed的30% */
        float diff_limit = (float)speed * ANGLE_DIFF_MAX_RATIO;
        if (diff_limit < 30.0f) diff_limit = 30.0f;  /* 最低限制30, 否则完全不转 */
        if (diff_f >  diff_limit) diff_f =  diff_limit;
        if (diff_f < -diff_limit) diff_f = -diff_limit;

        /* diff > 0 左转(减左轮), diff < 0 右转(减右轮) */
        int16_t left_target  = speed - (int16_t)diff_f;
        int16_t right_target = speed + (int16_t)diff_f;

        /* 限幅 */
        if (left_target < 0)    left_target = 0;
        if (right_target < 0)   right_target = 0;
        if (left_target > (int16_t)max_follow_spd)  left_target = (int16_t)max_follow_spd;
        if (right_target > (int16_t)max_follow_spd) right_target = (int16_t)max_follow_spd;

        /* 平滑输出 */
        Motor_SmoothDiff(left_target, right_target);

        /* 转向灯 */
        if (angle_error > tolerance)       TurnSignal_Set(TURN_LEFT);
        else if (angle_error < -tolerance) TurnSignal_Set(TURN_RIGHT);
        else                               TurnSignal_Set(TURN_NONE);

        OLED_ShowString(0, 6, "FOLLOWING   ");
    }
}
