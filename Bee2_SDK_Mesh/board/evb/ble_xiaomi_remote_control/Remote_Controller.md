# 遥控器快速配对

## 遥控器操作步骤

遥控器首先需要发送配对广播（Object ID 0x0002），Combo模组收到该信息后，交由应用层确认设备信息，若应用层确认连接该遥控器，则开始连接该设备并获取遥控器的Key信息。

## Realtek示例代码

在工程Options -> C++ -> Define 中修改产品PRODUCT_ID

driver_init();

初始化按键GPIO，定义按键中断，低功耗功能参考原厂demo

ble_fastpair_event();

发送快速配对广播包，修改宏ADV_FAST_PAIR_TIME和ADV_INTERVAL_TIME可定义快速配对广播时长与间隔

mibeacon_obj_enque_oneshot();

发送加密按键obj，修改宏OBJ_ADV_TIMEOUT_MS和OBJ_ADV_INTERVAL_MS可定义按键obj广播时长与间隔