#include "Motion_control.h"

AS5600_soft_IIC_many MC_AS5600;
uint32_t AS5600_SCL[] = {PB15, PB14, PB13, PB12}; //BMCU-V06-第三次验证
uint32_t AS5600_SDA[] = {PD0, PC15, PC14, PC13};
#define AS5600_PI 3.1415926535897932384626433832795
#define speed_filter_k 100
float speed_as5600[4] = {0, 0, 0, 0};

void MC_PULL_ONLINE_init()
{
    ADC_DMA_init();
}

float MC_PULL_stu_raw[4] = {0, 0, 0, 0};
int MC_PULL_stu[4] = {0, 0, 0, 0};
float MC_ONLINE_key_stu_raw[4] = {0, 0, 0, 0};
// 0-离线 1-在线双微动触发 2-外触发 3-内触发
int MC_ONLINE_key_stu[4] = {0, 0, 0, 0};

// 电压控制相关常量
float PULL_voltage_up = 1.85f;   // 状态 压力高 红灯
float PULL_voltage_down = 1.45f; // 状态 压力低 蓝灯
// 微动触发控制相关常量
float MC_PULL_voltage_pull = 1.5f;
bool Assist_send_filament[4] = {false, false, false, false};
bool pull_state_old = false; // 上次触发状态——True：未触发，False：进料完成
bool is_backing_out = false;
uint64_t Assist_filament_time[4] = {0, 0, 0, 0};
uint64_t Assist_send_time = 1200; // 仅触发外侧后，送料时长
// 退料距离 单位 MM
float_t P1X_OUT_filament_meters = 200.0f; // 内置200mm 外置700mm
float_t last_total_distance[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 初始化退料开始时的距离
// bool filament_channel_inserted[4]={false,false,false,false};//通道是否插入
// 使用双微动
bool is_two = true;

void MC_PULL_ONLINE_read()//在线读
{
    //4路线性霍尔数据 OH49E-S
    float *data = ADC_DMA_get_value();
    MC_PULL_stu_raw[3] = data[0];
    MC_ONLINE_key_stu_raw[3] = data[1];
    MC_PULL_stu_raw[2] = data[2];
    MC_ONLINE_key_stu_raw[2] = data[3];
    MC_PULL_stu_raw[1] = data[4];
    MC_ONLINE_key_stu_raw[1] = data[5];
    MC_PULL_stu_raw[0] = data[6];
    MC_ONLINE_key_stu_raw[0] = data[7];

    for (int i = 0; i < 4; i++)
    {
        /*
        if (i == 0){
            DEBUG_MY("MC_PULL_stu_raw = ");
            DEBUG_float(MC_PULL_stu_raw[i],3);
            DEBUG_MY("  MC_ONLINE_key_stu_raw = ");
            DEBUG_float(MC_ONLINE_key_stu_raw[i],3);
            DEBUG_MY("  通道：");
            DEBUG_float(i,1);
            DEBUG_MY("   \n");
        }
        */
        if (MC_PULL_stu_raw[i] > PULL_voltage_up) // 大于1.85V,表示压力过高
        {
            MC_PULL_stu[i] = 1;
        }
        else if (MC_PULL_stu_raw[i] < PULL_voltage_down) // 小于1.45V，表示压力过低
        {
            MC_PULL_stu[i] = -1;
        }
        else // 1.4~1.7之间，在正常误差范围内，无需动作
        {
            MC_PULL_stu[i] = 0;
        }
        /*在线状态*/

        // 耗材在线判断
        if (is_two == false)
        {
            // 大于1.65V，为耗材在线，高电平.
            if (MC_ONLINE_key_stu_raw[i] > 1.65)
            {
                MC_ONLINE_key_stu[i] = 1;
            }
            else
            {
                MC_ONLINE_key_stu[i] = 0;
            }
        }
        else
        {
            // DEBUG_MY(MC_ONLINE_key_stu_raw);
            // 双微动
            if (MC_ONLINE_key_stu_raw[i] < 0.6f)
            { // 小于则离线.
                MC_ONLINE_key_stu[i] = 0;
            }
            else if ((MC_ONLINE_key_stu_raw[i] < 1.7f) & (MC_ONLINE_key_stu_raw[i] > 1.4f))
            { // 仅触发外侧微动，需辅助进料
                MC_ONLINE_key_stu[i] = 2;
            }
            else if (MC_ONLINE_key_stu_raw[i] > 1.7f)
            { // 双微动同时触发, 在线状态
                MC_ONLINE_key_stu[i] = 1;
            }
            else if (MC_ONLINE_key_stu_raw[i] < 1.4f)
            { // 仅触发内侧微动 , 需确认是缺料还是抖动.
                MC_ONLINE_key_stu[i] = 3;
            }
        }
    }
}

#define PWM_lim 1000
//---------参数保存开始---------------
//电机 结构体?
struct alignas(4) Motion_control_save_struct
{
    int Motion_control_dir[4];
    int check = 0x40614061;//校验
} Motion_control_data_save;
//保存的闪存地址
#define Motion_control_save_flash_addr ((uint32_t)0x0800E000)
bool Motion_control_read()
{
    Motion_control_save_struct *ptr = (Motion_control_save_struct *)(Motion_control_save_flash_addr);
    if (ptr->check == 0x40614061)
    {
        memcpy(&Motion_control_data_save, ptr, sizeof(Motion_control_save_struct));
        return true;
    }
    return false;
}
void Motion_control_save()
{
    Flash_saves(&Motion_control_data_save, sizeof(Motion_control_save_struct), Motion_control_save_flash_addr);
}
//----------参数保存结束----------
//PID类
class MOTOR_PID
{

    float P = 0;
    float I = 0;
    float D = 0;
    float I_save = 0;
    float E_last = 0;
    float pid_MAX = PWM_lim;
    float pid_MIN = -PWM_lim;
    float pid_range = (pid_MAX - pid_MIN) / 2;

public:
    MOTOR_PID()
    {
        pid_MAX = PWM_lim;
        pid_MIN = -PWM_lim;
        pid_range = (pid_MAX - pid_MIN) / 2;
    }
    MOTOR_PID(float P_set, float I_set, float D_set)
    {
        init_PID(P_set, I_set, D_set);
        pid_MAX = PWM_lim;
        pid_MIN = -PWM_lim;
        pid_range = (pid_MAX - pid_MIN) / 2;
    }
    void init_PID(float P_set, float I_set, float D_set) // 注意，采用了PID独立的计算方法，I和D默认已乘P
    {
        P = P_set;
        I = I_set;
        D = D_set;
        I_save = 0;
        E_last = 0;
    }
    float caculate(float E, float time_E)
    {
        I_save += I * E * time_E;
        if (I_save > pid_range) // 对I限幅
            I_save = pid_range;
        if (I_save < -pid_range)
            I_save = -pid_range;

        float ouput_buf;
        if (time_E != 0) // 防止快速调用
            ouput_buf = P * E + I_save + D * (E - E_last) / time_E;
        else
            ouput_buf = P * E + I_save;

        if (ouput_buf > pid_MAX)
            ouput_buf = pid_MAX;
        if (ouput_buf < pid_MIN)
            ouput_buf = pid_MIN;

        E_last = E;
        return ouput_buf;
    }
    void clear()
    {
        I_save = 0;
        E_last = 0;
    }
};

//电机控制 参数 结构体
enum class filament_motion_enum
{
    filament_motion_send,
    filament_motion_redetect,
    filament_motion_slow_send,
    filament_motion_pull,
    filament_motion_stop,
    filament_motion_pressure_ctrl_on_use,
    filament_motion_pressure_ctrl_idle,
};
enum class pressure_control_enum
{
    less_pressure,
    all,
    over_pressure
};
//电机控制
class _MOTOR_CONTROL
{
public:
    filament_motion_enum motion = filament_motion_enum::filament_motion_stop;
    int CHx = 0;
    uint64_t motor_stop_time = 0;
    MOTOR_PID PID_speed = MOTOR_PID(2, 20, 0);
    MOTOR_PID PID_pressure = MOTOR_PID(1500, 0, 0);
    float pwm_zero = 500;
    float dir = 0;
    int x1 = 0;
    _MOTOR_CONTROL(int _CHx)
    {
        CHx = _CHx;
        motor_stop_time = 0;
        motion = filament_motion_enum::filament_motion_stop;
    }

    void set_pwm_zero(float _pwm_zero)
    {
        pwm_zero = _pwm_zero;
    }
    void set_motion(filament_motion_enum _motion, uint64_t over_time)
    {
        uint64_t time_now = get_time64();
        motor_stop_time = time_now + over_time;
        if (motion != _motion)
        {
            motion = _motion;
            PID_speed.clear();
        }
    }
    filament_motion_enum get_motion()
    {
        return motion;
    }
    float _get_x_by_pressure(float pressure_voltage, float control_voltage, float time_E, pressure_control_enum control_type)
    {
        float x=0;
        switch (control_type)
        {
        case pressure_control_enum::all: // 全范围控制
        {
            x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - control_voltage, time_E);
            break;
        }
        case pressure_control_enum::less_pressure: // 仅低压控制
        {
            if (pressure_voltage < control_voltage)
            {
                x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - control_voltage, time_E);
            }
            break;
        }
        case pressure_control_enum::over_pressure: // 仅高压控制
        {
            if (pressure_voltage > control_voltage)
            {
                x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - control_voltage, time_E);
            }
            break;
        }
        }
        if (x > 0) // 将控制力转为平方增强，平方会消掉正负，需要判断
            x = x * x / 250;
        else
            x = -x * x / 250;
        return x;
    }
    void run(float time_E)
    {
        // 当处于退料状态，并且需要退料时，开始记录里程。
        if (is_backing_out){
            last_total_distance[CHx] += fabs(speed_as5600[CHx] * time_E);
        }
        float speed_set = 0;
        float now_speed = speed_as5600[CHx];
        float x=0;

        uint16_t device_type = get_now_BambuBus_device_type();
        static uint64_t countdownStart[4] = {0};          // 辅助进料倒计时
        if (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle) // 在空闲状态
        {
            // 当 两个微动都被释放
            if (MC_ONLINE_key_stu[CHx] == 0)
            {
                Assist_send_filament[CHx] = true; // 某通道离线后才可触发辅助进料一次
                countdownStart[CHx] = 0;          // 清空倒计时
            }

            if (Assist_send_filament[CHx] && is_two)
            { // 允许状态，尝试辅助进料
                if (MC_ONLINE_key_stu[CHx] == 2)
                {                   // 触发外侧微动
                    x = -dir * 666; // 驱动送料
                }
                if (MC_ONLINE_key_stu[CHx] == 1)
                { // 同时触发双微动，准备停机
                    if (countdownStart[CHx] == 0)
                    { // 启动倒计时
                        countdownStart[CHx] = get_time64();
                    }
                    uint64_t now = get_time64();
                    if (now - countdownStart[CHx] >= Assist_send_time) // 倒计时
                    {
                        x = 0;                             // 停止电机
                        Assist_send_filament[CHx] = false; // 达成条件，完成一轮辅助进料。
                    }
                    else
                    {
                        // 驱动送料
                        x = -dir * 666;
                    }
                }
            }
            else
            {
                // 已经触发过，或微动触发在其他状态
                if (MC_ONLINE_key_stu[CHx] != 0 && MC_PULL_stu[CHx] != 0)
                { // 如果滑块被人为拉动，做出对应响应
                    x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - 1.65, time_E);
                }
                else
                { // 否则，保持停机
                    x = 0;
                    PID_pressure.clear();
                }
            }
        }
        else if (MC_ONLINE_key_stu[CHx] != 0) // 通道在运行状态，并且有耗材
        {
            if (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use) // 在使用状态
            {
                if (pull_state_old) { // 首次进入使用中，不触发后退，冲刷会让缓冲归位.
                    if (MC_PULL_stu_raw[CHx] < 1.55){
                        pull_state_old = false; // 检测到耗材已处于低压力。
                    }
                } else {
                    if (MC_PULL_stu_raw[CHx] < 1.65)
                    {
                        x = _get_x_by_pressure(MC_PULL_stu_raw[CHx], 1.65, time_E, pressure_control_enum::less_pressure);
                    }
                    else if (MC_PULL_stu_raw[CHx] > 1.7)
                    {
                        x = _get_x_by_pressure(MC_PULL_stu_raw[CHx], 1.7, time_E, pressure_control_enum::over_pressure);
                    }
                }
            }
            else
            {
                if (motion == filament_motion_enum::filament_motion_stop) // 要求停止
                {
                    PID_speed.clear();
                    Motion_control_set_PWM(CHx, 0);
                    return;
                }
                if (motion == filament_motion_enum::filament_motion_send) // 送料中
                {
                    if (device_type == BambuBus_AMS_lite)
                    {
                        if (MC_PULL_stu_raw[CHx] < 1.8) // 压力主动到1.8V位置
                            speed_set = 50;
                        else
                            speed_set = 0; // 原版这里是 10
                    }
                    else
                    {
                        speed_set = 50; // P系全力以赴
                    }
                }
                if (motion == filament_motion_enum::filament_motion_slow_send) // 要求缓慢送料
                {
                    speed_set = 3;
                }
                if (motion == filament_motion_enum::filament_motion_pull) // 回抽
                {
                    speed_set = -50;
                }
                x = dir * PID_speed.caculate(now_speed - speed_set, time_E);
            }
        }
        else // 运行过程中耗材用完，需要停止电机控制
        {
            x = 0;
        }

        if (x > 10)
            x += pwm_zero;
        else if (x < -10)
            x -= pwm_zero;
        else
            x = 0;

        if (x > PWM_lim)
        {
            x = PWM_lim;
        }
        if (x < -PWM_lim)
        {
            x = -PWM_lim;
        }

        Motion_control_set_PWM(CHx, x);
    }
};
_MOTOR_CONTROL MOTOR_CONTROL[4] = {_MOTOR_CONTROL(0), _MOTOR_CONTROL(1), _MOTOR_CONTROL(2), _MOTOR_CONTROL(3)};

void Motion_control_set_PWM(uint8_t CHx, int PWM)//传递到硬件层控制电机的PWM
{
    uint16_t set1 = 0, set2 = 0;
    if (PWM > 0)
    {
        set1 = PWM;
    }
    else if (PWM < 0)
    {
        set2 = -PWM;
    }
    else // PWM==0
    {
        set1 = 1000;
        set2 = 1000;
    }
    switch (CHx)
    {
    case 3:
        TIM_SetCompare1(TIM2, set1);
        TIM_SetCompare2(TIM2, set2);
        break;
    case 2:
        TIM_SetCompare1(TIM3, set1);
        TIM_SetCompare2(TIM3, set2);
        break;
    case 1:
        TIM_SetCompare1(TIM4, set1);
        TIM_SetCompare2(TIM4, set2);
        break;
    case 0:
        TIM_SetCompare3(TIM4, set1);
        TIM_SetCompare4(TIM4, set2);
        break;
    }
}

int32_t as5600_distance_save[4] = {0, 0, 0, 0};
void AS5600_distance_updata()//读取as5600，更新相关的数据
{
    static uint64_t time_last = 0;
    uint64_t time_now;
    float T;
    do
    {
        time_now = get_time64();
    } while (time_now <= time_last); // T!=0
    T = (float)(time_now - time_last);
    MC_AS5600.updata_angle();
    for (int i = 0; i < 4; i++)
    {
        if ((MC_AS5600.online[i] == false))
        {
            as5600_distance_save[i] = 0;
            speed_as5600[i] = 0;
            continue;
        }

        int32_t cir_E = 0;
        int32_t last_distance = as5600_distance_save[i];
        int32_t now_distance = MC_AS5600.raw_angle[i];
        float distance_E;
        if ((now_distance > 3072) && (last_distance <= 1024))
        {
            cir_E = -4096;
        }
        else if ((now_distance <= 1024) && (last_distance > 3072))
        {
            cir_E = 4096;
        }

        distance_E = -(float)(now_distance - last_distance + cir_E) * AS5600_PI * 7.5 / 4096; // D=7.5mm，加负号是因为AS5600正对磁铁
        as5600_distance_save[i] = now_distance;

        float speedx = distance_E / T * 1000;
        // T = speed_filter_k / (T + speed_filter_k);
        speed_as5600[i] = speedx; // * (1 - T) + speed_as5600[i] * T; // mm/s
        add_filament_meters(i, distance_E / 1000);
    }
    time_last = time_now;
}

enum filament_now_position_enum
{
    filament_idle,
    filament_sending_out,
    filament_using,
    filament_pulling_back,
    filament_redetect,
};
int filament_now_position[4];
bool wait = false;

bool Prepare_For_filament_Pull_Back(float_t OUT_filament_meters)
{
    bool wait = false;
    for (int i = 0; i < 4; i++)
    {
        if (filament_now_position[i] == filament_pulling_back)
        {
            // DEBUG_MY("last_total_distance: "); // 输出调试信息
            // Debug_log_write_float(last_total_distance[i], 5);
            if (last_total_distance[i] < OUT_filament_meters)
            {
                // 未到达时进行退料
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_pull, 100); // 驱动电机退料
                // 渐变灯效
                float npercent = (last_total_distance[i] / OUT_filament_meters) * 100.0f;
                MC_STU_RGB_set(i, 255 - ((255 / 100) * npercent), 125 - ((125 / 100) * npercent), (255 / 100) * npercent);
                // 退料未完成需要优先处理
            }
            else
            {
                // 到达停止距离
                is_backing_out = false; // 无需继续记录距离
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100); // 停止电机
                filament_now_position[i] = filament_idle;               // 设置当前位置为空闲
                set_filament_motion(i, AMS_filament_motion::idle);      // 强制进入空闲
                last_total_distance[i] = 0;                             // 重置退料距离
                // 退料完成
            }
            // 只要在退料状态就必须等待，直到不在退料中，下次循环后才不需要等待。
            wait = true;
        }
    }
    return wait;
}
void motor_motion_switch() // 通道状态切换函数，只控制当前在使用的通道，其他通道设置为停止
{
    int num = get_now_filament_num();                      // 当前通道号
    uint16_t device_type = get_now_BambuBus_device_type(); // 设备类型
    for (int i = 0; i < 4; i++)
    {
        if (i != num)
        {
            filament_now_position[i] = filament_idle;
            MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 1000);
        }
        else if (MC_ONLINE_key_stu[num] == 1 || MC_ONLINE_key_stu[num] == 3) // 通道有耗材丝
        {
            switch (get_filament_motion(num)) // 判断模拟器状态
            {
            case AMS_filament_motion::need_send_out: // 需要进料
                MC_STU_RGB_set(num, 00, 255, 00);
                filament_now_position[num] = filament_sending_out;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_send, 100);
                break;
            case AMS_filament_motion::need_pull_back:
                pull_state_old = false; // 重置标记
                is_backing_out = true; // 标记正在回退
                filament_now_position[num] = filament_pulling_back;
                if (device_type == BambuBus_AMS_lite)
                {
                    MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pull, 100);
                }
                // Prepare_For_filament_Pull_Back(OUT_filament_meters); // 通过距离控制退料是否完成
                break;
            case AMS_filament_motion::before_pull_back:
            case AMS_filament_motion::on_use:
            {
                static uint64_t time_end = 0;
                uint64_t time_now = get_time64();
                if (filament_now_position[num] == filament_sending_out) // 如果通道刚开始进料
                {
                    is_backing_out = false; // 设置无需记录距离
                    pull_state_old = true; // 首次不会往后拽，会等待触发低电压位，避免刚进入料就被拉出。
                    filament_now_position[num] = filament_using; // 标记为使用中
                    time_end = time_now + 1500;                  // 防止未被咬合, 持续进1.5秒
                }
                else if (filament_now_position[num] == filament_using) // 已经触发且处于使用中
                {
                    last_total_distance[i] = 0; // 重置退料距离
                    if (time_now > time_end)
                    {                                          // 已超1.5秒，进入通道使用 进行续料
                        MC_STU_RGB_set(num, 255, 255, 255); // 白色
                        MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_on_use, 20);
                    }
                    else
                    {                                                                  // 是否刚被检测到耗材丝
                        MC_STU_RGB_set(num, 128, 192, 128);                         // 淡绿色
                        MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_slow_send, 100); // 前1.5秒 慢速进料，辅助工具头咬合。
                    }
                }
                break;
            }
            case AMS_filament_motion::idle:
                filament_now_position[num] = filament_idle;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 100);
                for (int i = 0; i < 4; i++)
                {
                    // 硬件正常
                    if (MC_ONLINE_key_stu[i] == 1 || MC_ONLINE_key_stu[i] == 0)
                    {   // 同时触发或者无耗材丝
                        MC_STU_RGB_set(i, 0, 0, 255); // 蓝色
                    }
                    else if (MC_ONLINE_key_stu[i] == 2)
                    {   // 仅外部触发
                        MC_STU_RGB_set(i, 255, 144, 0); // 橙 /像金色
                    }
                    else if (MC_ONLINE_key_stu[i] == 3)
                    {   // 仅内部触发
                        MC_STU_RGB_set(i, 0, 255, 255); // 青色
                    }
                }
                break;
            }
        }
        else if (MC_ONLINE_key_stu[num] == 0) // 0:一定没有耗材丝，1:同时触发一定有耗材丝 2:仅外部触发 3:仅内部触发，这里有防掉线功能
        {
            filament_now_position[num] = filament_idle;
            MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 100);
            // MC_STU_RGB_set(num, 0, 0, 255);
        }
    }
}
// 根据AMS模拟器的信息，来调度电机
void motor_motion_run(int error)
{
    uint64_t time_now = get_time64();
    static uint64_t time_last = 0;
    float time_E = time_now - time_last; // 获取时间差（ms）
    time_E = time_E / 1000;              // 切换到单位s
    uint16_t device_type = get_now_BambuBus_device_type();
    if (!error) // 正常模式
    {
        // 根据设备类型执行不同的电机控制逻辑
        if (device_type == BambuBus_AMS_lite)
        {
            motor_motion_switch(); // 调度电机
        }
        else if (device_type == BambuBus_AMS)
        {
            if (!Prepare_For_filament_Pull_Back(P1X_OUT_filament_meters)) // 取反(返回true)，则代表不需要优先考虑退料，并继续调度电机。
            {
                motor_motion_switch(); // 调度电机
            }
        }
    }
    else // error模式
    {
        for (int i = 0; i < 4; i++)
            MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100); // 关闭电机
    }

    for (int i = 0; i < 4; i++)
    {
        /*if (!get_filament_online(i)) // 通道不在线则电机不允许工作
            MOTOR_CONTROL[i].set_motion(filament_motion_stop, 100);*/
        MOTOR_CONTROL[i].run(time_E); // 根据状态信息来驱动电机

        if (MC_PULL_stu[i] == 1)
        {
            MC_PULL_ONLINE_RGB_set(i, 255, 0, 0); // 压力过大，红灯
        }
        else if (MC_PULL_stu[i] == 0)
        { // 正常压力
            if (MC_ONLINE_key_stu[i] == 1)
            { // 在线，并且双微动触发
                int filament_colors_R = channel_colors[i][0];
                int filament_colors_G = channel_colors[i][1];
                int filament_colors_B = channel_colors[i][2];
                // 根据储存的耗材丝颜色
                MC_PULL_ONLINE_RGB_set(i, filament_colors_R, filament_colors_G, filament_colors_B);
                // 亮白灯
                // MC_STU_RGB_set(i, 255, 255, 255);
            }
            else
            {
                MC_PULL_ONLINE_RGB_set(i, 0, 0, 0); // 无耗材，不亮灯
            }
        }
        else if (MC_PULL_stu[i] == -1)
        {
            MC_PULL_ONLINE_RGB_set(i, 0, 0, 255); // 压力过小，蓝灯
        }
    }
    time_last = time_now;
}
// 运动控制函数
void Motion_control_run(int error)
{
    MC_PULL_ONLINE_read();

    AS5600_distance_updata();
    for (int i = 0; i < 4; i++)
    {
        if (MC_ONLINE_key_stu[i] == 0) {
            set_filament_online(i, false);
        } else if (MC_ONLINE_key_stu[i] == 1) {
            set_filament_online(i, true);
        } else if (MC_ONLINE_key_stu[i] == 3 && filament_now_position[i] == filament_using) {
            // 如果 仅内侧触发且在使用中，先不离线
            set_filament_online(i, true);
        } else if (filament_now_position[i] == filament_redetect || (filament_now_position[i] == filament_pulling_back)) {
            // 如果 处于退料返回，或退料中，先不离线
            set_filament_online(i, true);
        } else {
            set_filament_online(i, false);
        }
    }
    /*
        如果外侧微动触发，橙/ 像金色
        如果仅内测微动触发，// 青色
        如果同时触发，空闲 = 蓝色，同时代表耗材丝上线，蓝 + 白色/通道保存色
    */

    if (error) // Error != 0
    {
        for (int i = 0; i < 4; i++)
        {
            set_filament_online(i, false);
            // filament_channel_inserted[i] = true; // 用于测试
            if (MC_ONLINE_key_stu[i] == 1)
            {                                        // 同时触发
                MC_STU_RGB_set(i, 0, 0, 255); // 蓝色
            }
            else if (MC_ONLINE_key_stu[i] == 2)
            {                                        // 仅外部触发
                MC_STU_RGB_set(i, 255, 144, 0); // 橙/ 像金色
            }
            else if (MC_ONLINE_key_stu[i] == 3)
            {                                        // 仅内部触发
                MC_STU_RGB_set(i, 0, 255, 255); // 青色
            } else if (MC_ONLINE_key_stu[i] == 0)
            {   // 未连接打印机且没有耗材丝
                MC_STU_RGB_set(i, 0, 0, 0); // 黑色
            }
        }
    } else { // 正常连接到打印机
        // 在这里设置颜色会重复修改。
        for (int i = 0; i < 4; i++)
        {
            if ((MC_AS5600.online[i] == false) || (MC_AS5600.magnet_stu[i] == -1)) // AS5600 error
            {
                set_filament_online(i, false);
                MC_STU_ERROR[i] = true;
            }
        }
    }
    motor_motion_run(error);
}
// 设置PWM驱动电机
void MC_PWM_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//PB 3 4 5 6 7 8 9   BMCU V0.6

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//PA15
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // 开启复用时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // 开启TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // 开启TIM3时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // 开启TIM4时钟

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 定时器基础配置
    TIM_TimeBaseStructure.TIM_Period = 999;  // 周期（x+1）
    TIM_TimeBaseStructure.TIM_Prescaler = 1; // 预分频（x+1）
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // PWM模式配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // 占空比
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure); // PA15
    TIM_OC2Init(TIM2, &TIM_OCInitStructure); // PB3
    
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // PB4
    TIM_OC2Init(TIM3, &TIM_OCInitStructure); // PB5

    TIM_OC1Init(TIM4, &TIM_OCInitStructure); // PB6
    TIM_OC2Init(TIM4, &TIM_OCInitStructure); // PB7

    TIM_OC3Init(TIM4, &TIM_OCInitStructure); // PB8
    TIM_OC4Init(TIM4, &TIM_OCInitStructure); // PB9

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);    // TIM2完全映射-CH1-PA15/CH2-PB3 电机1
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); // TIM3部分映射-CH1-PB4/CH2-PB5 电机2
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);       // TIM4不映射-CH1-PB6/CH2-PB7/ 电机3   CH3-PB8/CH4-PB9  电机4

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}
// 获取PWM摩擦力零点（弃用，假设为50%占空比）
void MOTOR_get_pwm_zero()
{
    float pwm_zero[4] = {0, 0, 0, 0};
    MC_AS5600.updata_angle();

    int16_t last_angle[4];
    for (int index = 0; index < 4; index++)
    {
        last_angle[index] = MC_AS5600.raw_angle[index];
    }
    for (int pwm = 300; pwm < 1000; pwm += 10)
    {
        MC_AS5600.updata_angle();
        for (int index = 0; index < 4; index++)
        {

            if (pwm_zero[index] == 0)
            {
                if (abs(MC_AS5600.raw_angle[index] - last_angle[index]) > 50)
                {
                    pwm_zero[index] = pwm;
                    pwm_zero[index] *= 0.90;
                    Motion_control_set_PWM(index, 0);
                }
                else if ((MC_AS5600.online[index] == true))
                {
                    Motion_control_set_PWM(index, -pwm);
                }
            }
            else
            {
                Motion_control_set_PWM(index, 0);
            }
        }
        delay(100);
    }
    for (int index = 0; index < 4; index++)
    {
        Motion_control_set_PWM(index, 0);
        MOTOR_CONTROL[index].set_pwm_zero(pwm_zero[index]);
    }
}
// 将角度数值转化为角度差数值
int M5600_angle_dis(int16_t angle1, int16_t angle2)
{

    int cir_E = angle1 - angle2;
    if ((angle1 > 3072) && (angle2 <= 1024))
    {
        cir_E = -4096;
    }
    else if ((angle1 <= 1024) && (angle2 > 3072))
    {
        cir_E = 4096;
    }
    return cir_E;
}

// 测试电机运动方向
void MOTOR_get_dir()
{
    int dir[4] = {0, 0, 0, 0};
    bool done = false;
    bool have_data = Motion_control_read();
    if (!have_data)
    {
        for (int index = 0; index < 4; index++)
        {
            Motion_control_data_save.Motion_control_dir[index] = 0;
        }
    }
    MC_AS5600.updata_angle(); //读取5600的初始角度值

    int16_t last_angle[4];
    for (int index = 0; index < 4; index++)
    {
        last_angle[index] = MC_AS5600.raw_angle[index];                  //将初始角度值记录下来
        dir[index] = Motion_control_data_save.Motion_control_dir[index]; //记录flash中的dir数据
    }
    //bool need_test = false; // 是否需要检测
    bool need_save = false; // 是否需要更新状态
    for (int index = 0; index < 4; index++)
    {
        if ((MC_AS5600.online[index] == true)) // 有5600，说明通道在线
        {
            if (Motion_control_data_save.Motion_control_dir[index] == 0) // 之前测试结果为0，需要测试
            {
                Motion_control_set_PWM(index, 1000); // 打开电机
                //need_test = true;                    // 设置需要测试
                need_save = true;                    // 有状态更新
            }
        }
        else
        {
            dir[index] = 0;   // 通道不在线，清空它的方向数据
            need_save = true; // 有状态更新
        }
    }
    int i = 0;
    while (done == false)
    {
        done = true;

        delay(10);                // 间隔10ms检测一次
        MC_AS5600.updata_angle(); // 更新角度数据

        if (i++ > 200) // 超过2s无响应
        {
            for (int index = 0; index < 4; index++)
            {
                Motion_control_set_PWM(index, 0);                       // 停止
                Motion_control_data_save.Motion_control_dir[index] = 0; // 方向设为0
            }
            break; // 跳出循环
        }
        for (int index = 0; index < 4; index++) // 遍历
        {
            if ((MC_AS5600.online[index] == true) && (Motion_control_data_save.Motion_control_dir[index] == 0)) // 对于新的通道
            {
                int angle_dis = M5600_angle_dis(MC_AS5600.raw_angle[index], last_angle[index]);
                if (abs(angle_dis) > 163) // 移动超过1mm
                {
                    Motion_control_set_PWM(index, 0); // 停止
                    if (angle_dis > 0)                // 这里AS600正对着磁铁，和背贴方向是反的
                    {
                        dir[index] = 1;
                    }
                    else
                    {
                        dir[index] = -1;
                    }
                }
                else
                {
                    done = false; // 没有移动。继续等待
                }
            }
        }
    }
    for (int index = 0; index < 4; index++) // 遍历四个电机
    {
        Motion_control_data_save.Motion_control_dir[index] = dir[index]; // 数据复制
    }
    if (need_save) // 如果需要保存数据
    {
        Motion_control_save(); // 数据保存
    }
}
// 初始化电机
// 指定方向
int first_boot = 1; // 1 表示第一次启动，用于执行仅启动时.
void set_motor_directions(int dir0, int dir1, int dir2, int dir3)
{
    Motion_control_data_save.Motion_control_dir[0] = dir0;
    Motion_control_data_save.Motion_control_dir[1] = dir1;
    Motion_control_data_save.Motion_control_dir[2] = dir2;
    Motion_control_data_save.Motion_control_dir[3] = dir3;

    Motion_control_save(); // 保存到闪存
}
void MOTOR_init()
{

    MC_PWM_init();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    MC_AS5600.init(AS5600_SCL, AS5600_SDA, 4);
    // MOTOR_get_pwm_zero();
    // 自动方向
    MOTOR_get_dir();

    // 固定电机方向用
    if (first_boot == 1)
    { // 首次启动
        // set_motor_directions(1 , 1 , 1 , 1 ); // 1为正转 -1为反转
        first_boot = 0;
    }
    for (int index = 0; index < 4; index++)
    {
        Motion_control_set_PWM(index, 0);
        MOTOR_CONTROL[index].set_pwm_zero(500);
        MOTOR_CONTROL[index].dir = Motion_control_data_save.Motion_control_dir[index];
    }
    MC_AS5600.updata_angle();
    for (int i = 0; i < 4; i++)
    {
        as5600_distance_save[i] = MC_AS5600.raw_angle[i];
    }
}
extern void RGB_update();
void Motion_control_init() // 初始化所有运动和传感器
{
    MC_PULL_ONLINE_init();
    MC_PULL_ONLINE_read();
    MOTOR_init();
    
    /*
    //这是一段阻塞的DEBUG代码
    while (1)
    {
        delay(10);
        MC_PULL_ONLINE_read();

        for (int i = 0; i < 4; i++)
        {
            MOTOR_CONTROL[i].set_motion(filament_motion_pressure_ctrl_on_use, 100);
            if (!get_filament_online(i)) // 通道不在线则电机不允许工作
                MOTOR_CONTROL[i].set_motion(filament_motion_stop, 100);
            MOTOR_CONTROL[i].run(0); // 根据状态信息来驱动电机
        }
        char s[100];
        int n = sprintf(s, "%d\n", (int)(MC_PULL_stu_raw[3] * 1000));
        DEBUG_num(s, n);
    }*/

    for (int i = 0; i < 4; i++)
    {
        // if(MC_AS5600.online[i])//用AS5600是否有信号来判断通道是否插入
        // {
        //     filament_channel_inserted[i]=true;
        // }
        // else
        // {
        //     filament_channel_inserted[i]=false;
        // }
        filament_now_position[i] = filament_idle;//将通道初始状态设置为空闲
    }
}
