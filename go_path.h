/**
 * @file    go_path.h
 * @author  PickingChip & Jackrainman
 * @brief   跑点算法 - 集成路径规划与轨迹规划
 * @version 2.0
 * @date    2025-12-08
 *
 * @description
 * 本模块提供底盘跑点功能：
 * - 路径规划（避障）
 * - 轨迹规划（调用 chassis_motion）
 * - 位置PID闭环
 * - 一键跑点接口
 */

#ifndef __GO_PATH_H
#define __GO_PATH_H

#include "pid/pid.h"
#include "motion/chassis_motion.h"
#include <stdbool.h>

/*============================================================================
 *                              配置宏
 *============================================================================*/

/**
 * @brief 障碍物数量（0 = 无障碍物，直接跑点）
 * 在使用前通过定义此宏来设置障碍物数量
 */
#ifndef GO_PATH_OBSTACLE_NUM
#define GO_PATH_OBSTACLE_NUM        0
#endif

/**
 * @brief 路径规划配置
 */
#define GO_PATH_MAX_WAYPOINTS       10          /**< 最大途经点数量 */
#define GO_PATH_OBSTACLE_MARGIN     200.0f      /**< 障碍物安全边距 mm */

/**
 * @brief PID反馈权重
 */
#define GO_PATH_PID_WEIGHT_XY       0.3f        /**< XY位置PID权重 */
#define GO_PATH_PID_WEIGHT_YAW      0.5f        /**< Yaw位置PID权重 */

/*============================================================================
 *                              类型定义
 *============================================================================*/

/**
 * @brief 跑点状态
 */
typedef enum {
    GO_PATH_TARGET_NO_ARRIVE = 0,   /**< 未到达 */
    GO_PATH_TARGET_ARRIVE,          /**< 已到达 */
    GO_PATH_TARGET_POINT_TYPE_ERR,  /**< 点位类型错误 */
    GO_PATH_RUNNING,                /**< 运行中 */
} go_path_arrive_status_t;

/**
 * @brief 点位类型
 */
typedef enum {
    POINT_TYPE_ACTION_FLAT = 0,     /**< ACTION定位平跑 */
    POINT_TYPE_NUC_FLAT,            /**< NUC定位平跑 */
    POINT_TYPE_DT35,                /**< DT35定位 */
    POINT_TYPE_TARGET_RADIUM,       /**< 目标半径点 */

    POINT_TYPE_NUM
} go_path_point_type_t;

/**
 * @brief 定位类型
 */
typedef enum {
    LOCATION_TYPE_ACTION = 0,       /**< ACTION定位 */
    LOCATION_TYPE_DT35,             /**< DT35定位 */
    LOCATION_TYPE_NUC,              /**< NUC定位 */

    LOCATION_TYPE_NUM
} go_path_location_type_t;

/**
 * @brief 解算结果
 */
typedef struct {
    float moving_velocity;          /**< 平动速度 */
    float turning_velocity;         /**< 旋转速度 */
    float speed_angle;              /**< 平动速度与世界坐标轴夹角 */
    go_path_arrive_status_t arrived;/**< 到达状态 */
} go_path_result_t;

/**
 * @brief 二维点
 */
typedef struct {
    float x;
    float y;
} go_path_point2d_t;

/**
 * @brief 四边形障碍物
 */
typedef struct {
    go_path_point2d_t vertices[4];  /**< 4个顶点（顺/逆时针） */
} go_path_obstacle_t;

/**
 * @brief 路径结构
 */
typedef struct {
    go_path_point2d_t points[GO_PATH_MAX_WAYPOINTS];    /**< 途经点 */
    uint8_t count;                  /**< 点数量 */
    uint8_t current_index;          /**< 当前目标点索引 */
    bool valid;                     /**< 路径是否有效 */
} go_path_path_t;

/**
 * @brief 底盘控制函数指针
 */
typedef void (*go_path_chassis_func_t)(float, float, float);

/*============================================================================
 *                              全局变量
 *============================================================================*/

extern go_path_result_t go_path_result;
extern go_path_chassis_func_t go_path_chassis_ctrl;

#if GO_PATH_OBSTACLE_NUM > 0
extern go_path_obstacle_t go_path_obstacles[GO_PATH_OBSTACLE_NUM];
#endif

/*============================================================================
 *                              初始化接口
 *============================================================================*/

/**
 * @brief 注册底盘控制函数
 */
void go_path_chassis_ctrl_init(go_path_chassis_func_t chassis_ctrl_func);

/**
 * @brief 初始化定位源
 * @param location_type 定位类型
 * @param pos_x X坐标指针
 * @param pos_y Y坐标指针
 * @param pos_yaw Yaw角度指针
 */
void go_path_location_init(go_path_location_type_t location_type,
                           float *pos_x, float *pos_y, float *pos_yaw);

/**
 * @brief 初始化跑点PID
 * @param speed_pid 平动PID
 * @param angle_pid 转动PID
 * @param distance_deadband 平动死区 (mm)
 * @param angle_deadband 角度死区 (deg)
 * @param point_type 点位类型
 * @param location_type 定位类型
 */
void go_path_pidpoint_init(pid_t *speed_pid, pid_t *angle_pid,
                           float distance_deadband, float angle_deadband,
                           go_path_point_type_t point_type,
                           go_path_location_type_t location_type);

/*============================================================================
 *                              障碍物管理接口
 *============================================================================*/

/**
 * @brief 设置矩形障碍物
 * @param index 障碍物索引 (0 ~ GO_PATH_OBSTACLE_NUM-1)
 * @param x_min, y_min 左下角坐标
 * @param x_max, y_max 右上角坐标
 */
void go_path_set_obstacle_rect(uint8_t index,
                               float x_min, float y_min,
                               float x_max, float y_max);

/**
 * @brief 设置四边形障碍物
 * @param index 障碍物索引
 * @param x1,y1 ~ x4,y4 四个顶点坐标（顺/逆时针）
 */
void go_path_set_obstacle_quad(uint8_t index,
                               float x1, float y1, float x2, float y2,
                               float x3, float y3, float x4, float y4);

/**
 * @brief 检查点是否在障碍物内
 */
bool go_path_is_point_blocked(float x, float y);

/**
 * @brief 检查线段是否穿过障碍物
 */
bool go_path_is_line_blocked(float x1, float y1, float x2, float y2);

/*============================================================================
 *                              路径规划接口
 *============================================================================*/

/**
 * @brief 规划路径（自动避障）
 * @param start_x, start_y 起点坐标
 * @param goal_x, goal_y 终点坐标
 * @param path 输出：规划的路径
 * @return true=规划成功, false=失败
 */
bool go_path_plan(float start_x, float start_y,
                  float goal_x, float goal_y,
                  go_path_path_t *path);

/*============================================================================
 *                              跑点接口
 *============================================================================*/

/**
 * @brief 角度优化（选择劣弧）
 */
float angle_trans(float self_angle, float target_angle);

/**
 * @brief 原有跑点接口（纯PID，兼容旧代码）
 *
 * @param target_x, target_y 目标坐标 (mm)
 * @param target_yaw 目标角度 (deg)
 * @param point_type 点位类型
 * @return 到达状态
 */
go_path_arrive_status_t go_path_by_point(float target_x, float target_y,
                                         float target_yaw,
                                         go_path_point_type_t point_type);

/**
 * @brief 一键跑点（路径规划 + 轨迹规划 + PID闭环）
 *
 * 自动处理：
 * 1. 路径规划（如果有障碍物）
 * 2. 轨迹规划（缓加速/减速）
 * 3. PID闭环修正
 * 4. 输出速度（不直接控制底盘）
 *
 * @param target_x, target_y, target_yaw 目标点
 * @param point_type 点位类型
 * @param out_vx, out_vy, out_vw 输出：控制速度
 * @return 状态
 */
go_path_arrive_status_t go_path_auto_run(float target_x, float target_y,
                                         float target_yaw,
                                         go_path_point_type_t point_type,
                                         float *out_vx, float *out_vy,
                                         float *out_vw);

/**
 * @brief 重置自动跑点状态（切换目标点时调用）
 */
void go_path_auto_reset(void);

/**
 * @brief 获取自动跑点进度 (0.0~1.0)
 */
float go_path_auto_get_progress(void);

#endif /* __GO_PATH_H */
