/**
 * @file    go_path.c
 * @author  PickingChip & Jackrainman
 * @brief   跑点算法 - 集成路径规划与轨迹规划
 * @version 2.0
 * @date    2025-12-08
 */

#include "go_path.h"
#include "my_math/my_math.h"
#include <math.h>
#include <string.h>

/*============================================================================
 *                              内部宏定义
 *============================================================================*/

#define EPSILON 0.001f
#define PI 3.14159265358979f

#ifndef fabsf
#define fabsf(x) ((x) >= 0 ? (x) : -(x))
#endif

#ifndef sqrtf
#define sqrtf(x) ((float)sqrt((double)(x)))
#endif

/*============================================================================
 *                              内部类型
 *============================================================================*/

/* PID类型索引 */
enum {
    SPEED_PID = 0,
    ANGLE_PID,
    PID_NUM,
};

/* 位置指针 */
typedef struct {
    float *chassis_x;
    float *chassis_y;
    float *chassis_yaw;
} pos_point_t;

/* 跑点状态 */
static struct {
    float target_x;
    float target_y;
    float target_yaw;
    go_path_location_type_t location_type;
    float distance_deadband;
    float angle_deadband;
    pos_point_t chassis_position;
    pid_t *running_pid[PID_NUM];
} go_path_state[POINT_TYPE_NUM];

/*============================================================================
 *                              全局变量
 *============================================================================*/

go_path_chassis_func_t go_path_chassis_ctrl = NULL;
pos_point_t location_pos[LOCATION_TYPE_NUM] = {0};
go_path_result_t go_path_result = {0};

#if GO_PATH_OBSTACLE_NUM > 0
go_path_obstacle_t go_path_obstacles[GO_PATH_OBSTACLE_NUM] = {0};
#endif

/* 自动跑点状态 */
static struct {
    go_path_path_t path;
    ChassisPositionPlanner planner;
    float last_target_x, last_target_y, last_target_yaw;
    float last_wp_x, last_wp_y;
    bool initialized;
    bool running;
} auto_state = {0};

/*============================================================================
 *                              辅助函数
 *============================================================================*/

/**
 * @brief 计算两点距离
 */
static float calc_distance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief 角度优化（选择劣弧）
 */
float angle_trans(float self_angle, float target_angle) {
    float tmp, res1, res2;
    tmp = self_angle - target_angle;
    res1 = tmp > 0 ? tmp - 360.0f : tmp + 360.0f;
    res2 = tmp;
    tmp = my_fabs(res1) < my_fabs(res2) ? res1 : res2;
    return tmp;
}

/**
 * @brief 叉积
 */
static float cross_product(float ox, float oy, float ax, float ay,
                           float bx, float by) {
    return (ax - ox) * (by - oy) - (ay - oy) * (bx - ox);
}

/**
 * @brief 判断两线段是否相交
 */
static bool segments_intersect(float p1x, float p1y, float p2x, float p2y,
                               float p3x, float p3y, float p4x, float p4y) {
    float d1 = cross_product(p3x, p3y, p4x, p4y, p1x, p1y);
    float d2 = cross_product(p3x, p3y, p4x, p4y, p2x, p2y);
    float d3 = cross_product(p1x, p1y, p2x, p2y, p3x, p3y);
    float d4 = cross_product(p1x, p1y, p2x, p2y, p4x, p4y);

    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
        ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
        return true;
    }
    return false;
}

/**
 * @brief 判断点是否在四边形内
 */
static bool point_in_quad(float px, float py, const go_path_obstacle_t *obs) {
    int count = 0;
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) % 4;
        float x1 = obs->vertices[i].x, y1 = obs->vertices[i].y;
        float x2 = obs->vertices[j].x, y2 = obs->vertices[j].y;

        if ((y1 <= py && y2 > py) || (y2 <= py && y1 > py)) {
            float x_int = x1 + (py - y1) / (y2 - y1) * (x2 - x1);
            if (px < x_int) {
                count++;
            }
        }
    }
    return (count % 2) == 1;
}

/*============================================================================
 *                              初始化接口
 *============================================================================*/

void go_path_chassis_ctrl_init(go_path_chassis_func_t chassis_ctrl_func) {
    go_path_chassis_ctrl = chassis_ctrl_func;
}

void go_path_location_init(go_path_location_type_t location_type,
                           float *pos_x, float *pos_y, float *pos_yaw) {
    if (location_type >= LOCATION_TYPE_NUM) return;

    location_pos[location_type].chassis_x = pos_x;
    location_pos[location_type].chassis_y = pos_y;
    location_pos[location_type].chassis_yaw = pos_yaw;
}

void go_path_pidpoint_init(pid_t *speed_pid, pid_t *angle_pid,
                           float distance_deadband, float angle_deadband,
                           go_path_point_type_t point_type,
                           go_path_location_type_t location_type) {
    if (point_type >= POINT_TYPE_NUM) return;
    if (location_type >= LOCATION_TYPE_NUM) return;

    go_path_state[point_type].target_x = 0.0f;
    go_path_state[point_type].target_y = 0.0f;
    go_path_state[point_type].target_yaw = 0.0f;
    go_path_state[point_type].distance_deadband = distance_deadband;
    go_path_state[point_type].angle_deadband = angle_deadband;
    go_path_state[point_type].location_type = location_type;
    go_path_state[point_type].running_pid[SPEED_PID] = speed_pid;
    go_path_state[point_type].running_pid[ANGLE_PID] = angle_pid;
    go_path_state[point_type].chassis_position.chassis_x =
        location_pos[location_type].chassis_x;
    go_path_state[point_type].chassis_position.chassis_y =
        location_pos[location_type].chassis_y;
    go_path_state[point_type].chassis_position.chassis_yaw =
        location_pos[location_type].chassis_yaw;
}

/*============================================================================
 *                              障碍物管理
 *============================================================================*/

#if GO_PATH_OBSTACLE_NUM > 0

void go_path_set_obstacle_rect(uint8_t index,
                               float x_min, float y_min,
                               float x_max, float y_max) {
    if (index >= GO_PATH_OBSTACLE_NUM) return;

    go_path_obstacles[index].vertices[0].x = x_min;
    go_path_obstacles[index].vertices[0].y = y_min;
    go_path_obstacles[index].vertices[1].x = x_max;
    go_path_obstacles[index].vertices[1].y = y_min;
    go_path_obstacles[index].vertices[2].x = x_max;
    go_path_obstacles[index].vertices[2].y = y_max;
    go_path_obstacles[index].vertices[3].x = x_min;
    go_path_obstacles[index].vertices[3].y = y_max;
    go_path_obstacles[index].enabled = true;
}

void go_path_set_obstacle_quad(uint8_t index,
                               float x1, float y1, float x2, float y2,
                               float x3, float y3, float x4, float y4) {
    if (index >= GO_PATH_OBSTACLE_NUM) return;

    go_path_obstacles[index].vertices[0].x = x1;
    go_path_obstacles[index].vertices[0].y = y1;
    go_path_obstacles[index].vertices[1].x = x2;
    go_path_obstacles[index].vertices[1].y = y2;
    go_path_obstacles[index].vertices[2].x = x3;
    go_path_obstacles[index].vertices[2].y = y3;
    go_path_obstacles[index].vertices[3].x = x4;
    go_path_obstacles[index].vertices[3].y = y4;
    go_path_obstacles[index].enabled = true;
}

void go_path_enable_obstacle(uint8_t index, bool enable) {
    if (index >= GO_PATH_OBSTACLE_NUM) return;
    go_path_obstacles[index].enabled = enable;
}

void go_path_clear_obstacles(void) {
    memset(go_path_obstacles, 0, sizeof(go_path_obstacles));
}

bool go_path_is_point_blocked(float x, float y) {
    for (int i = 0; i < GO_PATH_OBSTACLE_NUM; i++) {
        if (!go_path_obstacles[i].enabled) continue;
        if (point_in_quad(x, y, &go_path_obstacles[i])) {
            return true;
        }
    }
    return false;
}

bool go_path_is_line_blocked(float x1, float y1, float x2, float y2) {
    for (int i = 0; i < GO_PATH_OBSTACLE_NUM; i++) {
        if (!go_path_obstacles[i].enabled) continue;

        /* 检查与四条边的相交 */
        for (int j = 0; j < 4; j++) {
            int k = (j + 1) % 4;
            if (segments_intersect(x1, y1, x2, y2,
                    go_path_obstacles[i].vertices[j].x,
                    go_path_obstacles[i].vertices[j].y,
                    go_path_obstacles[i].vertices[k].x,
                    go_path_obstacles[i].vertices[k].y)) {
                return true;
            }
        }

        /* 检查中点 */
        if (point_in_quad((x1 + x2) / 2, (y1 + y2) / 2, &go_path_obstacles[i])) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 获取途经点候选（障碍物顶点外扩）
 */
static int get_waypoint_candidates(go_path_point2d_t *candidates, int max_count) {
    int count = 0;

    for (int i = 0; i < GO_PATH_OBSTACLE_NUM && count < max_count; i++) {
        if (!go_path_obstacles[i].enabled) continue;

        /* 计算中心 */
        float cx = 0, cy = 0;
        for (int j = 0; j < 4; j++) {
            cx += go_path_obstacles[i].vertices[j].x;
            cy += go_path_obstacles[i].vertices[j].y;
        }
        cx /= 4;
        cy /= 4;

        /* 顶点外扩 */
        for (int j = 0; j < 4; j++) {
            float vx = go_path_obstacles[i].vertices[j].x;
            float vy = go_path_obstacles[i].vertices[j].y;

            float dx = vx - cx;
            float dy = vy - cy;
            float len = sqrtf(dx * dx + dy * dy);
            if (len > EPSILON) {
                dx /= len;
                dy /= len;
            }

            float wx = vx + dx * GO_PATH_OBSTACLE_MARGIN;
            float wy = vy + dy * GO_PATH_OBSTACLE_MARGIN;

            /* 检查是否有效 */
            if (!go_path_is_point_blocked(wx, wy)) {
                candidates[count].x = wx;
                candidates[count].y = wy;
                count++;
            }
        }
    }
    return count;
}

bool go_path_plan(float start_x, float start_y,
                  float goal_x, float goal_y,
                  go_path_path_t *path) {
    path->count = 0;
    path->current_index = 0;
    path->valid = false;

    /* 能否直达 */
    if (!go_path_is_line_blocked(start_x, start_y, goal_x, goal_y)) {
        path->points[0].x = goal_x;
        path->points[0].y = goal_y;
        path->count = 1;
        path->valid = true;
        return true;
    }

    /* 获取候选点 */
    go_path_point2d_t candidates[GO_PATH_OBSTACLE_NUM * 4];
    int candidate_count = get_waypoint_candidates(candidates, GO_PATH_OBSTACLE_NUM * 4);

    if (candidate_count == 0) {
        path->points[0].x = goal_x;
        path->points[0].y = goal_y;
        path->count = 1;
        path->valid = true;
        return true;
    }

    /* 贪心搜索 */
    float cur_x = start_x, cur_y = start_y;
    uint8_t used[GO_PATH_OBSTACLE_NUM * 4] = {0};

    for (int iter = 0; iter < GO_PATH_MAX_WAYPOINTS - 1; iter++) {
        if (!go_path_is_line_blocked(cur_x, cur_y, goal_x, goal_y)) {
            path->points[path->count].x = goal_x;
            path->points[path->count].y = goal_y;
            path->count++;
            path->valid = true;
            return true;
        }

        float best_score = 1e9f;
        int best_idx = -1;

        for (int i = 0; i < candidate_count; i++) {
            if (used[i]) continue;
            if (go_path_is_line_blocked(cur_x, cur_y, candidates[i].x, candidates[i].y)) {
                continue;
            }

            float d1 = calc_distance(cur_x, cur_y, candidates[i].x, candidates[i].y);
            float d2 = calc_distance(candidates[i].x, candidates[i].y, goal_x, goal_y);
            float score = d1 + d2;

            if (score < best_score) {
                best_score = score;
                best_idx = i;
            }
        }

        if (best_idx < 0) break;

        path->points[path->count].x = candidates[best_idx].x;
        path->points[path->count].y = candidates[best_idx].y;
        path->count++;
        used[best_idx] = 1;

        cur_x = candidates[best_idx].x;
        cur_y = candidates[best_idx].y;
    }

    path->points[path->count].x = goal_x;
    path->points[path->count].y = goal_y;
    path->count++;
    path->valid = true;

    return true;
}

#else /* GO_PATH_OBSTACLE_NUM == 0 */

void go_path_set_obstacle_rect(uint8_t index, float x_min, float y_min,
                               float x_max, float y_max) { (void)index; }
void go_path_set_obstacle_quad(uint8_t index, float x1, float y1, float x2, float y2,
                               float x3, float y3, float x4, float y4) { (void)index; }
void go_path_enable_obstacle(uint8_t index, bool enable) { (void)index; }
void go_path_clear_obstacles(void) {}
bool go_path_is_point_blocked(float x, float y) { return false; }
bool go_path_is_line_blocked(float x1, float y1, float x2, float y2) { return false; }

bool go_path_plan(float start_x, float start_y, float goal_x, float goal_y,
                  go_path_path_t *path) {
    path->points[0].x = goal_x;
    path->points[0].y = goal_y;
    path->count = 1;
    path->current_index = 0;
    path->valid = true;
    return true;
}

#endif

/*============================================================================
 *                              原有PID跑点
 *============================================================================*/

/**
 * @brief PID控制计算
 */
static void action_pid_control(go_path_point_type_t point_type,
                               float target_x, float target_y, float target_yaw) {
    bool arrive_xy = false;
    bool arrive_yaw = false;

    float cur_x = *go_path_state[point_type].chassis_position.chassis_x;
    float cur_y = *go_path_state[point_type].chassis_position.chassis_y;
    float cur_yaw = *go_path_state[point_type].chassis_position.chassis_yaw;

    /* 平动 */
    float delta_distance = calc_distance(cur_x, cur_y, target_x, target_y);

    if (delta_distance > go_path_state[point_type].distance_deadband) {
        go_path_result.moving_velocity =
            pid_calc(go_path_state[point_type].running_pid[SPEED_PID],
                     delta_distance, 0);

        /* 减速距离限制 */
        const ChassisMotionParams *params = chassis_motion_get_params();
        float max_v = sqrtf(2.0f * params->a_max_xy * delta_distance);
        if (go_path_result.moving_velocity > max_v) {
            go_path_result.moving_velocity = max_v;
        }

        /* 方向 */
        if ((target_y - cur_y) > 0) {
            go_path_result.speed_angle = acosf((target_x - cur_x) / delta_distance);
        } else {
            go_path_result.speed_angle = -acosf((target_x - cur_x) / delta_distance);
        }
        arrive_xy = false;
    } else {
        go_path_result.moving_velocity = 0.0f;
        go_path_result.speed_angle = 0.0f;
        go_path_state[point_type].running_pid[SPEED_PID]->iout = 0;
        arrive_xy = true;
    }

    /* 转动 */
    float delta_angle = angle_trans(cur_yaw, target_yaw);
    if (my_fabs(delta_angle) > go_path_state[point_type].angle_deadband) {
        go_path_result.turning_velocity =
            pid_calc(go_path_state[point_type].running_pid[ANGLE_PID],
                     delta_angle, 0);
        arrive_yaw = false;
    } else {
        go_path_result.turning_velocity = 0.0f;
        go_path_state[point_type].running_pid[ANGLE_PID]->iout = 0;
        arrive_yaw = true;
    }

    /* 状态 */
    if (arrive_xy && arrive_yaw) {
        go_path_result.arrived = GO_PATH_TARGET_ARRIVE;
    } else {
        go_path_result.arrived = GO_PATH_TARGET_NO_ARRIVE;
    }
}

go_path_arrive_status_t go_path_by_point(float target_x, float target_y,
                                         float target_yaw,
                                         go_path_point_type_t point_type) {
    if (point_type >= POINT_TYPE_NUM) {
        return GO_PATH_TARGET_POINT_TYPE_ERR;
    }

    go_path_state[point_type].target_x = target_x;
    go_path_state[point_type].target_y = target_y;
    go_path_state[point_type].target_yaw = target_yaw;

    action_pid_control(point_type, target_x, target_y, target_yaw);

    float speedx = go_path_result.moving_velocity * cosf(go_path_result.speed_angle);
    float speedy = go_path_result.moving_velocity * sinf(go_path_result.speed_angle);
    float speedw = go_path_result.turning_velocity;

    if (go_path_chassis_ctrl) {
        go_path_chassis_ctrl(speedx, speedy, speedw);
    }

    return go_path_result.arrived;
}

/*============================================================================
 *                              一键跑点
 *============================================================================*/

void go_path_auto_reset(void) {
    auto_state.initialized = false;
    auto_state.running = false;
    auto_state.path.current_index = 0;
    chassis_position_planner_reset(&auto_state.planner);
}

float go_path_auto_get_progress(void) {
    if (!auto_state.initialized) return 0;

    float path_progress = (float)auto_state.path.current_index / auto_state.path.count;
    float seg_progress = chassis_position_planner_get_progress(&auto_state.planner);

    return path_progress + seg_progress / auto_state.path.count;
}

go_path_arrive_status_t go_path_auto_run(float target_x, float target_y,
                                         float target_yaw,
                                         go_path_point_type_t point_type,
                                         float *out_vx, float *out_vy,
                                         float *out_vw) {
    if (point_type >= POINT_TYPE_NUM) {
        *out_vx = *out_vy = *out_vw = 0;
        return GO_PATH_TARGET_POINT_TYPE_ERR;
    }

    float cur_x = *go_path_state[point_type].chassis_position.chassis_x;
    float cur_y = *go_path_state[point_type].chassis_position.chassis_y;
    float cur_yaw = *go_path_state[point_type].chassis_position.chassis_yaw;

    /* 1. 检测目标变化，重新规划 */
    if (!auto_state.initialized ||
        fabsf(target_x - auto_state.last_target_x) > 10.0f ||
        fabsf(target_y - auto_state.last_target_y) > 10.0f ||
        fabsf(target_yaw - auto_state.last_target_yaw) > 1.0f) {

        go_path_plan(cur_x, cur_y, target_x, target_y, &auto_state.path);
        auto_state.path.current_index = 0;
        auto_state.last_target_x = target_x;
        auto_state.last_target_y = target_y;
        auto_state.last_target_yaw = target_yaw;
        auto_state.initialized = true;
        auto_state.running = true;

        /* 初始化到第一个点的轨迹 */
        float wp_x = auto_state.path.points[0].x;
        float wp_y = auto_state.path.points[0].y;
        bool is_last = (auto_state.path.count == 1);
        float wp_yaw = is_last ? target_yaw : cur_yaw;

        chassis_position_planner_init(&auto_state.planner,
                                      cur_x, cur_y, cur_yaw,
                                      wp_x, wp_y, wp_yaw,
                                      true);
        auto_state.last_wp_x = wp_x;
        auto_state.last_wp_y = wp_y;
    }

    /* 2. 检查路径完成 */
    if (auto_state.path.current_index >= auto_state.path.count) {
        *out_vx = *out_vy = *out_vw = 0;
        return GO_PATH_TARGET_ARRIVE;
    }

    /* 3. 获取当前途经点 */
    float wp_x = auto_state.path.points[auto_state.path.current_index].x;
    float wp_y = auto_state.path.points[auto_state.path.current_index].y;
    bool is_last = (auto_state.path.current_index == auto_state.path.count - 1);
    float wp_yaw = is_last ? target_yaw : cur_yaw;

    /* 4. 途经点变化，重新初始化轨迹 */
    if (fabsf(wp_x - auto_state.last_wp_x) > 1.0f ||
        fabsf(wp_y - auto_state.last_wp_y) > 1.0f) {
        chassis_position_planner_init(&auto_state.planner,
                                      cur_x, cur_y, cur_yaw,
                                      wp_x, wp_y, wp_yaw,
                                      true);
        auto_state.last_wp_x = wp_x;
        auto_state.last_wp_y = wp_y;
    }

    /* 5. 更新轨迹规划 */
    float p_des_x, p_des_y, p_des_yaw;
    float v_ff_x, v_ff_y, v_ff_yaw;
    bool traj_done = chassis_position_planner_update(&auto_state.planner,
                                                     &p_des_x, &p_des_y, &p_des_yaw,
                                                     &v_ff_x, &v_ff_y, &v_ff_yaw);

    /* 6. PID闭环修正 */
    action_pid_control(point_type, p_des_x, p_des_y, p_des_yaw);

    float v_pid_x = go_path_result.moving_velocity * cosf(go_path_result.speed_angle);
    float v_pid_y = go_path_result.moving_velocity * sinf(go_path_result.speed_angle);
    float v_pid_yaw = go_path_result.turning_velocity;

    /* 7. 前馈 + 反馈 */
    *out_vx = v_ff_x + v_pid_x * GO_PATH_PID_WEIGHT_XY;
    *out_vy = v_ff_y + v_pid_y * GO_PATH_PID_WEIGHT_XY;
    *out_vw = v_ff_yaw + v_pid_yaw * GO_PATH_PID_WEIGHT_YAW;

    /* 8. 途经点切换 */
    float dist_to_wp = calc_distance(cur_x, cur_y, wp_x, wp_y);

    if (dist_to_wp < go_path_state[point_type].distance_deadband || traj_done) {
        auto_state.path.current_index++;

        if (auto_state.path.current_index < auto_state.path.count) {
            float next_wp_x = auto_state.path.points[auto_state.path.current_index].x;
            float next_wp_y = auto_state.path.points[auto_state.path.current_index].y;
            bool next_is_last = (auto_state.path.current_index == auto_state.path.count - 1);
            float next_wp_yaw = next_is_last ? target_yaw : cur_yaw;

            chassis_position_planner_init(&auto_state.planner,
                                          cur_x, cur_y, cur_yaw,
                                          next_wp_x, next_wp_y, next_wp_yaw,
                                          true);
            auto_state.last_wp_x = next_wp_x;
            auto_state.last_wp_y = next_wp_y;
        } else {
            /* 最终检查角度 */
            float delta_angle = angle_trans(cur_yaw, target_yaw);
            if (my_fabs(delta_angle) < go_path_state[point_type].angle_deadband) {
                *out_vx = *out_vy = *out_vw = 0;
                return GO_PATH_TARGET_ARRIVE;
            }
        }
    }

    return GO_PATH_RUNNING;
}
