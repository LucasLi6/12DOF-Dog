#include "stm32f10x.h"
#include "Delay.h"
#include "BlueTooth.h"
#include "Servo.h"
#include "PetAction.h"
#include "Face_Config.h"
#include "wexcube.h"
#include "RobotState.h"
#include "robot_config.h"
#include "error_handler.h"

// 舵机角度安全范围（根据实际机械结构调整）
#define MIN_LEG_HEIGHT 140
#define MAX_LEG_HEIGHT 180

// 错误处理函数（无OLED显示）
void handle_error(ErrorCode_t error)
{
    // 安全停止所有动作
    robot.action_mode = 1;
    robot.action_state = ACTION_IDLE;
    
    // 设置舵机到安全位置（中间位置）
    int i;
    for (i = 1; i <= 12; i++) {
        switch(i) {
            case 1: Servo_Angle1(90); break;
            case 2: Servo_Angle2(90); break;
            case 3: Servo_Angle3(90); break;
            case 4: Servo_Angle4(90); break;
            case 5: Servo_Angle5(90); break;
            case 6: Servo_Angle6(90); break;
            case 7: Servo_Angle7(90); break;
            case 8: Servo_Angle8(90); break;
            case 9: Servo_Angle9(90); break;
            case 10: Servo_Angle10(90); break;
            case 11: Servo_Angle11(90); break;
            case 12: Servo_Angle12(90); break;
        }
    }
}

// 全局变量
RobotState_t robot = {
    .height = 160,
    .action_mode = 0,
    .face_mode = 0,
    .action_high = 1,        // 初始值会在 main 中被强制设为 0
    .speed_delay = 10,
    .swing_delay = 50,
    .wei_ba = 0,
    .action_state = ACTION_IDLE,
    .leg_positions = {
        .x = {0, 0, 0, 0},
        .y = {INITIAL_Y_POS, INITIAL_Y_POS, INITIAL_Y_POS, INITIAL_Y_POS},
        .z = {0, 0, 0, 0}
    }
};

uint32_t g_SystemTime = 0;

// 存储小程序滑块值
static uint8_t wex_p = 0;      // P（ID22）
static uint8_t wex_i = 0;      // I（ID23）
static uint8_t wex_d = 0;      // D（ID24）
static uint8_t wex_fb = 50;    // 前后重心（ID25）
static uint8_t wex_lr = 50;    // 左右重心（ID26）
static uint8_t wex_ud = 50;    // 上下重心（ID27）——保留但踏步动作不再使用
static uint8_t height_slider_raw = 50; // 新增：ID27滑块原始值，用于踏步高度控制

// 动作映射表
typedef struct {
    uint8_t ctrl_id;
    uint8_t face_mode;
    uint8_t action_mode;
} ActionMap_t;

const ActionMap_t action_map[] = {
    {1, 0, 0},   // 趴下
    {2, 1, 1},   // 停止
    {3, 5, 2},   // 站立
    {4, 1, 3},   // 坐下
    {5, 2, 4},   // 前进
    {6, 2, 5},   // 后退
    {7, 2, 6},   // 左转
    {8, 2, 7},   // 右转
    {9, 4, 8},   // 摇摆
    {10,4, 9},   // 左前走
    {11,4,10},   // 左后走
    {12,4,11},   // 右前走
    {13,4,12},   // 右后走
    {14,4,13},   // 右走
    {15,4,14},   // 左走
    {16,4,15},   // 测试
    {17,4,16},   // 跳舞
    {20,4,17},   // 踏步
    {21,4,18},   // 撒尿
    {0,0,0}
};

// 动作执行变量
ActionState currentActionState = ACTION_IDLE;
uint32_t lastActionTime = 0;
uint32_t actionInterval = ACTION_INTERVAL_MS;
uint32_t actionStartTime = 0;

// 函数声明
void set_legs_neutral(void);

//------------------------------------------------------------------------------
// 设置所有腿到中立位置（前后x=0，左右y=0，高度=robot.height）
void set_legs_neutral(void)
{
    x1_y1(0, 0, robot.height);
    Delay_ms(1);
    x2_y2(0, 0, robot.height);
    Delay_ms(1);
    x3_y3(0, 0, robot.height);
    Delay_ms(1);
    x4_y4(0, 0, robot.height);
    Delay_ms(1);
}

//------------------------------------------------------------------------------
// 非阻塞动作执行器（由主循环定期调用）
void executeActionNonBlocking(void)
{
    static uint8_t prev_action_mode = 0xFF;  // 记录上一次模式，用于检测切换
    // 新增状态变量，用于跳舞和撒尿动作的状态机
    static uint32_t danceStartTime = 0;
    static uint8_t danceStep = 0;
    static uint32_t peeStartTime = 0;
    static uint8_t peeStep = 0;

    // 踏步模式专用保存/恢复变量
    static uint8_t last_action_mode = 0xFF;  // 上一次的模式（用于检测进入/退出踏步）
    static float saved_height = 0;            // 保存进入踏步前的 robot.height
    static uint8_t saved_action_high = 0;     // 保存进入踏步前的 robot.action_high

    // 检测模式切换
    if (robot.action_mode != prev_action_mode) {
        prev_action_mode = robot.action_mode;
        if (robot.action_mode == 2) {          // 进入站立模式
            set_legs_neutral();                 // 改为中立姿态
            currentActionState = ACTION_COMPLETED;
        } else if (robot.action_mode == 1) {   // 进入停止模式
            set_legs_neutral();
            currentActionState = ACTION_COMPLETED;
        } else if (robot.action_mode == 16) {  // 进入跳舞模式，重置状态
            danceStartTime = 0;
            danceStep = 0;
        } else if (robot.action_mode == 18) {  // 进入撒尿模式，重置状态
            peeStartTime = 0;
            peeStep = 0;
        }
        // 其他模式无需特殊初始化
    }

    // 踏步模式进入/退出检测（独立于上面的通用切换，因为需要保存/恢复高度）
    if (robot.action_mode != last_action_mode) {
        if (robot.action_mode == 17) {          // 进入踏步模式
            // 保存当前高度和 action_high
            saved_height = robot.height;
            saved_action_high = robot.action_high;
        } else if (last_action_mode == 17) {    // 退出踏步模式
            // 恢复之前保存的高度和 action_high
            robot.height = saved_height;
            robot.action_high = saved_action_high;
        }
        last_action_mode = robot.action_mode;
    }

    // 空闲状态直接返回
    if (currentActionState == ACTION_IDLE) return;

    // 超时检测（仅对非站立模式生效，跳过循环动作）
    if (robot.action_mode != 2 && robot.action_mode != 16 && robot.action_mode != 18) {
        if (g_SystemTime - actionStartTime > ACTION_TIMEOUT_MS) {
            currentActionState = ACTION_CANCELLED;
            handle_error(ERROR_ACTION_TIMEOUT);
            return;
        }
    }

    // 动作间隔控制
    if (g_SystemTime - lastActionTime < actionInterval) return;
    lastActionTime = g_SystemTime;

    // 根据当前模式执行动作
    switch (robot.action_mode) {
        case 0: // 趴下
            Action_dongzhuo_1(-15,-15,-15,-15,130,130,130,130,10,0);
            currentActionState = ACTION_COMPLETED;
            break;
        case 1: // 停止（已在切换时执行立正，此处无需重复）
            // 保持中立，不执行任何操作
            break;
        case 2: // 站立（已在中立姿态，无需重复操作）
            break;
        case 3: // 坐下
            Action_dongzhuo_1(-15,-15,-15,-15,170,170,130,130,10,3);
            currentActionState = ACTION_COMPLETED;
            break;
        case 4: // 前进（使用 robot.height，由ID18/19控制）
            Action_advance(50,55,12,robot.height,3,4,1,1,1,4);
				 //步长60、 步高、x轴前后重心（+中心前移）、腿高、 z轴左右重心(+身体中心左移) 、时序 、前进1后退-1 左转-1：1、右转1：1	、中断
            break;
        case 5: // 后退（使用 robot.height）
            Action_advance(50,55,12,robot.height,3,4,-1,1,1,5);
            break;
        case 6: // 左转
            Action_advance_yuandizhuan(50,50,12,robot.height,3,4,1,1,1,6);
            break;
        case 7: // 右转
            Action_advance_yuandizhuan(50,50,12,robot.height,3,4,-1,1,1,7);
            break;
        case 8: // 摇摆
            Action_yaobai(50,30,12,robot.height,3,25,1,1,1,8);
            break;
        case 9: // 左前走
            Action_advance(50,55,12,robot.height,3,4,1,0.7,1,9);
            break;
        case 10: // 左后走
            Action_advance(50,55,12,robot.height,3,4,-1,0.7,1,10);
            break;
        case 11: // 右前走
            Action_advance(50,55,12,robot.height,3,4,1,1,0.7,11);
            break;
        case 12: // 右后走
            Action_advance(50,55,12,robot.height,3,4,-1,1,0.7,12);
            break;
        case 13: // 右走
            Action_advance_zuoyou(35,55,12,robot.height,3,4,1,1,1,13);
            break;
        case 14: // 左走
            Action_advance_zuoyou(35,55,12,robot.height,3,4,-1,1,1,14);
            break;
        case 15: // 测试
            Action_ceshi();
            currentActionState = ACTION_COMPLETED;
            break;
        case 16: // 跳舞
            if (danceStartTime == 0) {
                danceStartTime = g_SystemTime;
                danceStep = 0;
            }
            
            // 跳舞序列 - 使用大狗坐标
            if (danceStep == 0 && g_SystemTime - danceStartTime >= 1000) {
                Action_dongzhuo_2(
                    20, 20, 20, 20, //x1 x2 x3 x4
                    robot.height, robot.height, robot.height, robot.height, //y1 y2 y3 y4
                    0, 0, 0, 0, 2, 16);//z1 z2 z3 z4
                danceStep = 1;
                danceStartTime = g_SystemTime;
            } else if (danceStep == 1 && g_SystemTime - danceStartTime >= 1000) {
                Action_dongzhuo_2(20, 20, 20, 20, 130, robot.height, robot.height, robot.height, 30, 0, 0, 0, 1, 16);
                danceStep = 2;
                danceStartTime = g_SystemTime;
            } else if (danceStep == 2 && g_SystemTime - danceStartTime >= 1000) {
                Action_dongzhuo_2(20, 20, 20, 20, robot.height, robot.height, robot.height, robot.height, 0, 0, 0, 0, 1, 16);
                danceStep = 3;
                danceStartTime = g_SystemTime;
            } else if (danceStep == 3 && g_SystemTime - danceStartTime >= 1000) {
                Action_dongzhuo_2(20, 20, 20, 20, robot.height, 130, robot.height, robot.height, 0, 30, 0, 0, 1, 16);
                danceStep = 4;
                danceStartTime = g_SystemTime;
            } else if (danceStep == 4 && g_SystemTime - danceStartTime >= 1000) {
                Action_dongzhuo_2(20, 20, 20, 20, robot.height, robot.height, robot.height, robot.height, 0, 0, 0, 0, 1, 16);
                danceStep = 5;
                danceStartTime = g_SystemTime;
            } else if (danceStep == 5 && g_SystemTime - danceStartTime >= 1000) {
                Action_dongzhuo_2(-30, -30, -30, -30, robot.height, robot.height, robot.height, robot.height, 0, 0, 0, 0, 2, 16);
                danceStep = 6;
                danceStartTime = g_SystemTime;
            } else if (danceStep == 6 && g_SystemTime - danceStartTime >= 1000) {
                Action_dongzhuo_2(-30, -30, -30, -30, robot.height, robot.height, 130, robot.height, 0, 0, -30, 0, 2, 16);
                danceStep = 7;
                danceStartTime = g_SystemTime;
            } else if (danceStep == 7 && g_SystemTime - danceStartTime >= 1000) {
                Action_dongzhuo_2(-30, -30, -30, -30, robot.height, robot.height, robot.height, robot.height, 0, 0, 0, 0, 1, 16);
                danceStep = 8;
                danceStartTime = g_SystemTime;
            } else if (danceStep == 8 && g_SystemTime - danceStartTime >= 1000) {
                Action_dongzhuo_2(-30, -30, -30, -30, robot.height, robot.height, robot.height, 130, 0, 0, 0, -30, 1, 16);
                danceStep = 9;
                danceStartTime = g_SystemTime;
            } else if (danceStep == 9 && g_SystemTime - danceStartTime >= 1000) {
                Action_dongzhuo_2(-30, -30, -30, -30, robot.height, robot.height, robot.height, robot.height, 0, 0, 0, 0, 1, 16);
                danceStep = 10;
                danceStartTime = g_SystemTime;
            } else if (danceStep == 10 && g_SystemTime - danceStartTime >= 1000) {
                Action_dongzhuo_2(0, 0, 0, 0, robot.height, robot.height, robot.height, robot.height, 0, 0, 0, 0, 2, 16);
                danceStep = 0;
                danceStartTime = g_SystemTime;
            } 
            break;
        case 17: // 踏步 —— 使用滑块独立控制高度
            {
                // 将滑块值0~100线性映射到140~180
                uint8_t target_height = 140 + (height_slider_raw * 40 / 100);
                // 强制使用滑块高度，并确保 action_high=0 以便 tiao_high() 返回 robot.height
                robot.height = target_height;
                robot.action_high = 0;
                // 调用踏步动作函数（mm 参数会被内部 tiao_high() 覆盖，但我们已经设置了 robot.height）
                Action_advance(0, 55, wex_fb - 50, target_height, wex_lr - 50, 4, 1, 1, 1, 17);
            }
            break;
        case 18: // 撒尿
            if (peeStartTime == 0) {
                peeStartTime = g_SystemTime;
                peeStep = 0;
            }
            
            // 撒尿序列 - 使用大狗坐标
            if (peeStep == 0 && g_SystemTime - peeStartTime >= 1000) {
                // 准备姿势：后腿稍微弯曲
                Action_dongzhuo_2(-15, -15, -15, -15, robot.height, robot.height, robot.height, robot.height, 0, 0, 0, 0, 2, 18);
                peeStep = 1;
                peeStartTime = g_SystemTime;
            } else if (peeStep == 1 && g_SystemTime - peeStartTime >= 1000) {
                // 开始撒尿：后腿弯曲，身体前倾
                Action_dongzhuo_2(-20, -20, -20, -20, robot.height, robot.height, robot.height, robot.height, 0, 0, 0, 0, 1, 18);
                peeStep = 2;
                peeStartTime = g_SystemTime;
            } else if (peeStep == 2 && g_SystemTime - peeStartTime >= 2000) {
                // 撒尿中：保持姿势
                Action_dongzhuo_2(-50, -50, -50, -50, robot.height, robot.height, robot.height, 110, 0, 0, 0, 60, 1, 18);
                peeStep = 3;
                peeStartTime = g_SystemTime;
            } else if (peeStep == 3 && g_SystemTime - peeStartTime >= 2000) {
                // 撒尿中：稍微调整姿势
                Action_dongzhuo_2(-45, -45, -45, -45, robot.height, robot.height, robot.height, 115, 0, 0, 0, 55, 1, 18);
                peeStep = 4;
                peeStartTime = g_SystemTime;
            } else if (peeStep == 4 && g_SystemTime - peeStartTime >= 2000) {
                // 结束撒尿：恢复站姿
                Action_dongzhuo_2(-15, -15, -15, -15, robot.height, robot.height, robot.height, robot.height, 0, 0, 0, 0, 2, 18);
                peeStep = 0;
                peeStartTime = g_SystemTime;
            }
            break;
        default:
            currentActionState = ACTION_COMPLETED;
            break;
    }
}

//------------------------------------------------------------------------------
int main(void)
{
    int i;
    const t_sWexCmd *psWexCmd;

    Servo_Init();
    BlueTooth_Init();
    wex_init();
    wex_start();
    
    robot.face_mode = 5;
    robot.action_mode = 2;          // 默认站立
    robot.action_high = 0;           // 强制使用 robot.height 作为高度源（关键修改）
    currentActionState = ACTION_RUNNING;
    actionStartTime = g_SystemTime;
    
    Delay_ms(100);                  // 初始化等待
    
    // 初始站立（设为中立姿态）
    set_legs_neutral();
    currentActionState = ACTION_COMPLETED;

    while(1)
    {
        psWexCmd = wex_process();
        
        if (psWexCmd->eCmdType != eWexCmd_None)
        {
            switch (psWexCmd->eCmdType)
            {
            case eWexCmd_Event:
            case eWexCmd_Value:
                // 更新滑块值（ID22~27）
                if (psWexCmd->ucCtrlId == 22) {
                    wex_p = psWexCmd->ucValue;
                } else if (psWexCmd->ucCtrlId == 23) {
                    wex_i = psWexCmd->ucValue;
                } else if (psWexCmd->ucCtrlId == 24) {
                    wex_d = psWexCmd->ucValue;
                } else if (psWexCmd->ucCtrlId == 25) {
                    wex_fb = psWexCmd->ucValue;
                } else if (psWexCmd->ucCtrlId == 26) {
                    wex_lr = psWexCmd->ucValue;
                } else if (psWexCmd->ucCtrlId == 27) {    // 上下重心滑块
                    wex_ud = psWexCmd->ucValue;            // 保留原变量（可能被其他地方引用）
                    height_slider_raw = psWexCmd->ucValue; // 新变量，专用于踏步高度
                }
                // 处理身高加减按钮 (ID18, ID19)
                else if (psWexCmd->ucCtrlId == 18) { // 身高加
                    if (psWexCmd->ucValue == 1) {
                        robot.height += 10;
                        if (robot.height > MAX_LEG_HEIGHT) robot.height = MAX_LEG_HEIGHT;
                        robot.action_high = 0; // 强制使用自定义高度
                    }
                } else if (psWexCmd->ucCtrlId == 19) { // 身高减
                    if (psWexCmd->ucValue == 1) {
                        if (robot.height >= 10) robot.height -= 10;
                        else robot.height = MIN_LEG_HEIGHT;
                        if (robot.height < MIN_LEG_HEIGHT) robot.height = MIN_LEG_HEIGHT;
                        robot.action_high = 0; // 强制使用自定义高度
                    }
                } else {
                    // 动作按钮处理（包含21撒尿）
                    for (i = 0; action_map[i].ctrl_id != 0; i++) {
                        if (psWexCmd->ucCtrlId == action_map[i].ctrl_id && psWexCmd->ucValue == 1) {
                            robot.face_mode = action_map[i].face_mode;
                            robot.action_mode = action_map[i].action_mode;
                            currentActionState = ACTION_RUNNING;
                            actionStartTime = g_SystemTime;
                            break;
                        }
                    }
                }
                break;
            default:
                break;
            }
        }
        
        // 执行非阻塞动作
        executeActionNonBlocking();
        
        // 系统时间更新，添加溢出检查
        if (g_SystemTime < 0xFFFFFFFF) {
            g_SystemTime += 1;           // 保持1ms增量
        } else {
            // 时间溢出，重置系统时间
            g_SystemTime = 0;
            // 重置所有时间相关变量
            lastActionTime = 0;
            actionStartTime = 0;
        }
        
        Delay_ms(1);                  // 保持1ms延时，使循环周期接近1ms
    }
}