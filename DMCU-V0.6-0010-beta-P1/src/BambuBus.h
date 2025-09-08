#pragma once

#include "main.h"

#define Bambubus_version 5 //Bmcu协议版本

#ifdef __cplusplus
extern "C"
{
#endif

    enum class AMS_filament_stu  //AMS耗材 灯丝
    {
        offline,//离线
        online,//在线
        NFC_waiting //NFC 等待
    };
    enum class AMS_filament_motion //AMS耗材运动
    {
        before_pull_back,//拉回之前
        need_pull_back,//需要拉回来
        need_send_out,//需要发送
        on_use,//使用
        idle//闲置
    };
    enum class BambuBus_package_type  //总线包类型
    {
        ERROR = -1,//错误
        NONE = 0,//无
        filament_motion_short,//耗材运动短
        filament_motion_long,//耗材运动长
        online_detect,//在线检测
        REQx6,//REQ x6
        NFC_detect,//NFC 检测
        set_filament_info,//设置耗材信息
        MC_online,//MC 在线
        read_filament_info,//读取耗材信息
        set_filament_info_type2,//设置耗材信息类型2
        version,//版本
        serial_number,//序列号
        heartbeat,//心跳
        ETC,//等等
        __BambuBus_package_packge_type_size //软件包 软件包类型 大小
    };
    
    enum BambuBus_device_type //驱动类型
    {
        BambuBus_none = 0x0000,//未知没有
        BambuBus_AMS = 0x0700, //AMS1代
        BambuBus_AMS_lite = 0x1200,//AMS_lite A1
    };
    extern void BambuBus_init();//总线初始化
    extern BambuBus_package_type BambuBus_run(); //运行返回包类型
#define max_filament_num 4 //最大耗材数
    extern bool Bambubus_read(); //读取
    extern void Bambubus_set_need_to_save();//设置需要保存
    extern int get_now_filament_num();//获取当前耗材数量
    extern uint16_t get_now_BambuBus_device_type();//获取BambuBus设备类型
    extern void reset_filament_meters(int num);//复位耗材数据
    extern void add_filament_meters(int num, float meters);//添加耗材数据
    extern float get_filament_meters(int num);//获取耗材数据
    extern void set_filament_online(int num, bool if_online);//设置耗材在线
    extern bool get_filament_online(int num);//获取耗材在线
    AMS_filament_motion get_filament_motion(int num);//AMS耗材运动状态
    extern void set_filament_motion(int num, AMS_filament_motion motion);//设置AMS耗材运动状态
    extern bool BambuBus_if_on_print();//是否正在打印
#ifdef __cplusplus
}
#endif