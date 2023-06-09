# 米家BLE Object定义

*本文档仅供参考，MiBeacon 和 Object相关接口已全部封装，详见* [标准BLE接入开发](https://iot.mi.com/new/doc/embedded-development/ble/standard.html)

## 概述

本文档定义了MiBeacon使用场景中的Object ID以及Object格式。设备上报事件或属性时需要广播包含Object的MiBeacon。网关会解析有效的Object并上报给米家后台，可供APP显示或执行一系列的自动化操作。

格式定义

|      字段       | 长度  |    描述    |
| :-------------: | :---: | :--------: |
|    Object ID    |   2   | 米家定义ID |
| Object Data Len |   1   |  Data长度  |
|   Object Data   |   N   |    数据    |

### 网关限制

表示事件或属性的MiBeacon（包含Object的MiBeacon）都是通过网关上报给后台，因此网关对某些参数有限制。

- 同一账号下只能支持50款子设备上报信息。
- 当网关周边有多于200个能够广播的BLE设备时，网关的性能会受到影响。
- 不支持大于31 Bytes长度的MiBeacon。
- 每一款设备（每一款pid）最多只能支持7种Object。
- 每个MiBeacon中只能包含一个Object。
- 每个Object的有效数据长度最大为10 bytes。
- 某些较老版本的网关（MTK7697 1.x）只能支持大约15种品类的子设备。

### 蓝牙子设备申请接入蓝牙网关 ###

蓝牙子设备如果想申请加入蓝牙网关，可以发送包含**产品的Model** 和**上报的Object ID** 的邮件到tss-miot@xiaomi.com 进行申请。

蓝牙子设备接入蓝牙网关之后，可通过附近同账号下的蓝牙网关设备接入互联网，以便通过手机远程查看设备状态，也可以实现自动化联动。



## ID定义

由米家定义，分配如下：

- 0x0000 - 0x1001 为蓝牙通用事件
- 0x1002 - 0x1FFF 为蓝牙通用属性
- 0x2000 - 0x2FFF 为厂商自定义属性，由相应厂商向小米申请
- 0x3000 - 0x3FFF 为厂商自定义事件，由相应厂商向小米申请

为保证数据及时、有效，并且不要给米家服务器造成较大负载压力，米家BLE网关会依据ID对不同类型数据做过滤，当数据与之前上报数据相比相隔一定时间且超出一定变化量，BLE网关才会转发该数据至米家服务器。因此专门定义了时间间隔和变化量，如下表所示。

为了保证Object代表的事件和属性可以被米家BLE网关收到，包含Object的MiBeacon需要重发多次。具体参考米家BLE MiBeacon协议。

### 通用事件

|        名称        |   ID   | 时间间隔（秒） | 变化量 |
| :----------------: | :----: | :------------: | :----: |
|        连接        | 0x0001 |       0        |   0    |
|      简易配对      | 0x0002 |       0        |   0    |
|        靠近        | 0x0003 |       0        |   0    |
|        远离        | 0x0004 |       0        |   0    |
|    锁（已废弃）    | 0x0005 |       0        |   0    |
|        指纹        | 0x0006 |       0        |   0    |
|         门         | 0x0007 |       0        |   0    |
|        布防        | 0x0008 |       0        |   0    |
|     手势控制器     | 0x0009 |       0        |   0    |
|        体温        | 0x000A |       60       |   0    |
|         锁         | 0x000B |       0        |   0    |
|        水浸        | 0x000C |       0        |   0    |
|        烟雾        | 0x000D |       0        |   0    |
|        燃气        | 0x000E |       0        |   0    |
| 有人移动（带光照） | 0x000F |       0        |   0    |
|        按键        | 0x1001 |       0        |   0    |

### 通用属性

|    名称    |   ID   | 时间间隔（秒） | 变化量 |
| :--------: | :----: | :------------: | :----: |
|    睡眠    | 0x1002 |      600       |   0    |
|    RSSI    | 0x1003 |       0        |   10   |
|    温度    | 0x1004 |      600       |   1    |
|    湿度    | 0x1006 |      600       |   1    |
|   光照度   | 0x1007 |      600       |   1    |
|  土壤湿度  | 0x1008 |      600       |   1    |
|  土壤EC值  | 0x1009 |      600       |   1    |
|    电量    | 0x100A |      600       |   0    |
|     锁     | 0x100E |       60       |   0    |
|     门     | 0x100F |       60       |   0    |
|    甲醛    | 0x1010 |      600       |   1    |
|    绑定    | 0x1011 |      600       |   0    |
|    开关    | 0x1012 |      600       |   0    |
| 耗材剩余量 | 0x1013 |      600       |   0    |
|    水浸    | 0x1014 |      600       |   0    |
|    烟雾    | 0x1015 |      600       |   0    |
|    燃气    | 0x1016 |      600       |   0    |
|  无人移动  | 0x1017 |       0        |   0    |
|  光照强弱  | 0x1018 |       0        |   1    |

### 厂商自定义属性

|       名称       |   ID   | 时间间隔（秒） | 变化量 |
| :--------------: | :----: | :------------: | :----: |
|  体温（秒秒测）  | 0x2000 |      600       |   0    |
| 小米手环（华米） | 0x2001 |      600       |   1    |
|  吸尘器（睿米）  | 0x2002 |      600       |   0    |
| 黑加手环（如一） | 0x2003 |      600       |   0    |

### 厂商自定义事件

|        名称        |   ID   | 时间间隔（秒） | 变化量 |
| :----------------: | :----: | :------------: | :----: |
| 监测仪（花花草草） | 0x3000 |      600       |   1    |
| 传感器位置（青萍） | 0x3001 |      600       |   0    |

<br/>

## 事件格式定义

### 简易配对事件

|   名称    | 长度  |                  描述                   |
| :-------: | :---: | :-------------------------------------: |
| Object ID |   2   | 待配对Object ID，例如按键事件（0x1001） |

### 锁事件（已废弃）

|   名称   | 长度  |    描述    |
| :------: | :---: | :--------: |
|   操作   |   1   |   见下文   |
| 操作方式 |   1   |   见下文   |
|  Key ID  |   4   |   见下文   |
|  时间戳  |   4   | 操作时间戳 |

当前操作分为以下几类：

- 0x00：门外开锁
- 0x01：门外上锁
- 0x02：门内反锁
- 0x03：解除反锁
- 0x04：门内开锁
- 0x05：门内上锁
- 0xFF：异常

操作方式分为以下几类：

- 0x00：蓝牙
- 0x01：密码
- 0x02：指纹
- 0x03：钥匙
- 0x04：转盘
- 0x05：NFC
- 0x10：人工反锁
- 0x20：自动反锁
- 0xFF：异常

Key ID分为以下几类：

- 0x00000000：锁的管理员
- 0xFFFFFFFF：未知操作者
- 0x00000000 - 0x7FFFFFFF：蓝牙（最多 2147483647 个）
- 0x80010000 - 0x8001FFFF：指纹（最多 65536 个）
- 0x80020000 - 0x8002FFFF：密码（最多 65536 个）
- 0x80030000 - 0x8003FFFF：钥匙（最多 65536 个）
- 0x80040000 - 0x8004FFFF：NFC（最多 65536 个）

以0xC0DE起始的ID表示异常，外部触发的异常包括：

- 0xC0DE0000：错误密码频繁开锁
- 0xC0DE0001：错误指纹频繁开锁
- 0xC0DE0002：操作超时（密码输入超时）
- 0xC0DE0003：撬锁
- 0xC0DE0004：重置按键按下
- 0xC0DE0005：错误钥匙频繁开锁
- 0xC0DE0006：钥匙孔异物
- 0xC0DE0007：钥匙未取出
- 0xC0DE0008：错误NFC频繁开锁
- 0xC0DE0009：超时未按要求上锁

内部触发的异常包括：

- 0xC0DE1000：电量低于10%
- 0xC0DE1001：电量低于5%
- 0xC0DE1002：指纹传感器异常
- 0xC0DE1003：配件电池电量低

### 指纹事件

|   名称   | 长度  |  描述  |
| :------: | :---: | :----: |
|  Key ID  |   4   | 见上文 |
| 匹配结果 |   1   | 见下文 |

Key ID分为以下几类：

- 0x00000000：锁的管理员
- 0xFFFFFFFF：未知操作者

匹配结果分为以下几类：

- 0x00：匹配成功
- 0x01：匹配失败
- 0x02：超时未放入
- 0x03：质量低（太浅、模糊）
- 0x04：面积不足
- 0x05：皮肤太干
- 0x06：皮肤太湿

### 门事件

| 名称  | 长度  |  描述  |
| :---: | :---: | :----: |
| 状态  |   1   | 见下文 |

状态分为以下几类：

- 0x00：开门
- 0x01：关门
- 0x02：超时未关
- 0x03：敲门
- 0x04：撬门
- 0x05：门卡住

### 布防事件

|   名称   | 长度  |            描述            |
| :------: | :---: | :------------------------: |
| 是否启用 |   1   | 开启（0x01）、关闭（0x00） |
|  时间戳  |   4   |      UTC 时间（可选）      |

### 手势控制器事件

| 名称  | 长度  |  描述  |
| :---: | :---: | :----: |
| 手势  |   2   | 见下文 |

手势分为以下几类：

- 0x0001：摇一摇
- 0x0002：翻转90度
- 0x0003：翻转180度
- 0x0004：平面旋转
- 0x0005：敲击
- 0x0006：轻推

### 体温事件

| 名称  | 长度  |                            描述                            |
| :---: | :---: | :--------------------------------------------------------: |
| 体温  |   2   | 有符号型变量，单位0.01度，示例：A7 0E = 0x0EA7 表示37.51度 |

### 锁事件

|          名称          | 长度  |    描述    |
| :--------------------: | :---: | :--------: |
|          操作          |   1   |   见下文   |
| Key ID or Exception ID |   4   |   见下文   |
|         时间戳         |   4   | 操作时间戳 |

操作字段的低4位表示action，分为以下几类：

- 0000b：门外开锁
- 0001b：门外上锁
- 0010b：门内反锁
- 0011b：解除反锁
- 0100b：门内开锁
- 0101b：门内上锁
- 0110b：开启童锁
- 0111b：关闭童锁
- 1111b：异常

操作字段的高4位表示method，分为以下几类：

- 0000b：蓝牙方式
- 0001b：密码方式
- 0010b：生物特征（指纹、人脸、人体静脉、掌纹等）
- 0011b：钥匙方式
- 0100b：转盘方式
- 0101b：NFC方式
- 0110b：一次性密码
- 0111b：双重验证
- 1010b：人工
- 1011b：自动
- 1111b：异常

Key ID分为以下几类：

- 0x00000000：锁的管理员
- 0xFFFFFFFF：未知操作者
- 0xDEADBEEF：无效操作者
- 0x00000000 - 0x7FFFFFFF：蓝牙（最多 2147483647 个）
- 0x80010000 - 0x8001FFFF：生物特征-指纹（最多 65536 个）
- 0x80020000 - 0x8002FFFF：密码（最多 65536 个）
- 0x80030000 - 0x8003FFFF：钥匙（最多 65536 个）
- 0x80040000 - 0x8004FFFF：NFC（最多 65536 个）
- 0x80050000 - 0x8005FFFF：双重验证（最多 65536 个）
- 0x80060000 - 0x8006FFFF：生物特征-人脸（最多 65536 个）
- 0x80070000 - 0x8007FFFF：生物特征-指静脉（最多 65536 个）
- 0x80080000 - 0x8008FFFF：生物特征-掌纹（最多 65536 个）

以0xC0DE起始的ID表示异常，外部触发的异常包括：

- 0xC0DE0000：错误密码频繁开锁
- 0xC0DE0001：错误指纹频繁开锁
- 0xC0DE0002：操作超时（密码输入超时）
- 0xC0DE0003：撬锁
- 0xC0DE0004：重置按键按下
- 0xC0DE0005：错误钥匙频繁开锁
- 0xC0DE0006：钥匙孔异物
- 0xC0DE0007：钥匙未取出
- 0xC0DE0008：错误NFC频繁开锁
- 0xC0DE0009：超时未按要求上锁
- 0xC0DE000A：多种方式频繁开锁失败
- 0xC0DE000B：人脸频繁开锁失败
- 0xC0DE000C：静脉频繁开锁失败
- 0xC0DE000D：劫持报警
- 0xC0DE000E：布防后门内开锁

内部触发的异常包括：

- 0xC0DE1000：电量低于10%
- 0xC0DE1001：电量低于5%
- 0xC0DE1002：指纹传感器异常

### 水浸事件

|   名称    | 长度  |                  描述                  |
| :-------: | :---: | :------------------------------------: |
| 水浸事件  |   1   | 水浸报警（0x01）、水浸报警解除（0x00） |

### 烟雾事件

|   名称    | 长度  |                  描述                  |
| :-------: | :---: | :------------------------------------: |
| 烟雾事件  |   1   | 烟雾报警（0x01）、烟雾报警解除（0x00） |

### 燃气事件

|   名称    | 长度  |                      描述                      |
| :-------: | :---: | :--------------------------------------------: |
| 燃气事件  |   1   | 燃气泄漏报警（0x01）、燃气泄漏报警解除（0x00） |

### 有人移动事件（带光照）

|   名称    | 长度  |                      描述                      |
| :-------: | :---: | :--------------------------------------------: |
| 光照强度  |   3   |     光照强度单位为Lux，取值范围：0-120000      |

备注：此事件只针对同时带有光照传感器的人体传感器使用，单独的人体传感器可使用“靠近事件（0x0003）”

### 按键事件

| 名称  | 长度  |                   描述                   |
| :---: | :---: | :--------------------------------------: |
| Index |   2   |          按键编号，取值范围0~9           |
| 类型  |   1   | 单击（0x00）、双击（0x01）、长按（0x02） |

### 花花草草检测仪事件

| 名称  | 长度  |            描述            |
| :---: | :---: | :------------------------: |
| 状态  |   1   | 正常（0x00）、拔出（0x01） |

### 青萍传感器事件

| 名称  | 长度  |               描述               |
| :---: | :---: | :------------------------------: |
| 位置  |   1   | 与底座分离（0x00）、连接（0x01） |

<br/>

## 属性格式定义

### 睡眠属性

| 名称  | 长度  |             描述             |
| :---: | :---: | :--------------------------: |
| 状态  |   1   | 无睡眠（0x00）、睡着（0x01） |

### RSSI属性

| 名称  | 长度  |    描述    |
| :---: | :---: | :--------: |
| RSSI  |   1   | 信号强度值 |

### 温度属性

| 名称  | 长度  |                           描述                           |
| :---: | :---: | :------------------------------------------------------: |
| 温度  |   2   | 有符号型变量，单位0.1度，示例：1A 01 = 0x011A 表示28.2度 |

### 湿度属性

| 名称  | 长度  |                     描述                     |
| :---: | :---: | :------------------------------------------: |
| 湿度  |   2   | 湿度百分比，取值0-1000，例如346表示湿度34.6% |

### 光照度属性

| 名称  | 长度  |      描述      |
| :---: | :---: | :------------: |
|  Lux  |   3   | 范围：0-120000 |

### 土壤湿度属性

| 名称  | 长度  |          描述           |
| :---: | :---: | :---------------------: |
| 湿度  |   1   | 湿度百分比，范围：0-100 |

### 土壤EC属性

| 名称  | 长度  |          描述           |
| :---: | :---: | :---------------------: |
| EC值  |   2   | 单位us/cm，范围：0-5000 |

### 电量属性

|    名称    | 长度  |    描述     |
| :--------: | :---: | :---------: |
| 电量百分比 |   1   | 范围：0-100 |

### 锁属性

| 名称  | 长度  |  描述  |
| :---: | :---: | :----: |
| 状态  |   1   | 见下文 |

- bit 0：方舌状态（1：弹出；0：收回 ）
- bit 1：呆舌状态（1：弹出；0：收回）
- bit 2：斜舌状态（1：弹出；0：收回）
- bit 3：童锁状态（1：开启；0：关闭）

所有正常组合状态：

- 0x00：开锁状态（所有锁舌收回）
- 0x04：锁舌弹出（斜舌弹出）
- 0x05：上锁+锁舌弹出（方舌、斜舌弹出）
- 0x06：反锁+锁舌弹出（呆舌、斜舌弹出）
- 0x07：所有锁舌弹出（方舌、呆舌、斜舌弹出）

### 门属性

| 名称  | 长度  |                   描述                   |
| :---: | :---: | :--------------------------------------: |
| 状态  |   1   | 开门（0x00）、关门（0x01）、异常（0xFF） |

### 甲醛属性

|  名称  | 长度  |                       描述                        |
| :----: | :---: | :-----------------------------------------------: |
| 甲醛值 |   2   | 精度0.01mg/m3，示例：10 00 = 0x0010 表示0.16mg/m3 |

### 绑定属性

| 名称  | 长度  |              描述              |
| :---: | :---: | :----------------------------: |
| 状态  |   1   | 未绑定（0x00）、已绑定（0x01） |

### 开关属性

| 名称  | 长度  |            描述            |
| :---: | :---: | :------------------------: |
| 开关  |   1   | 关闭（0x00）、开启（0x01） |

### 耗材剩余量属性

|  名称  | 长度  |         描述          |
| :----: | :---: | :-------------------: |
| 剩余量 |   1   | 剩余百分比，范围0~100 |

### 水浸属性

|   名称   | 长度  |              描述              |
| :------: | :---: | :----------------------------: |
| 浸没状态 |   1   | 已浸没（0x01）、未浸没（0x00） |

### 烟雾属性

|   名称   | 长度  |              描述              |
| :------: | :---: | :----------------------------: |
| 烟雾状态 |   1   | 有烟雾（0x01）、无烟雾（0x00） |

### 燃气属性

|   名称   | 长度  |              描述              |
| :------: | :---: | :----------------------------: |
| 燃气状态 |   1   | 有泄漏（0x01）、无泄漏（0x00） |

### 无人移动属性

|   名称    | 长度  |               描述               |
| :-------: | :---: | :------------------------------: |
| 时间长度  |   4   | 无人移动状态的持续时间，单位为秒 |

备注：如果有人移动，需要上报时间为0的无人移动属性

### 光照强弱属性

|   名称    | 长度  |              描述              |
| :-------: | :---: | :----------------------------: |
| 光照强弱  |   1   | 光照强（0x01）、光照弱（0x00） |

备注：如果子设备能上报准确光照度，请使用光照度属性（0x1007）上报

### 秒秒测体温属性

|   名称   | 长度  |     描述      |
| :------: | :---: | :-----------: |
| 皮肤温度 |   2   | 精确到 0.01度 |
| PCB温度  |   2   | 精确到 0.01度 |
|   电量   |   1   |  电量百分比   |

### 华米手环属性

| 名称  | 长度  |            描述            |
| :---: | :---: | :------------------------: |
| 步数  |   2   |          当前步数          |
| 睡眠  |   1   | 入睡（0x01）、醒来（0x02） |
| RSSI  |   1   |     当前信号强度绝对值     |

### 睿米吸尘器属性

| 名称  | 长度  |                                 描述                                 |
| :---: | :---: | :------------------------------------------------------------------: |
| 模式  |   1   | 充电（0x00）、待机（0x01）、标准（0x02）、强劲（0x03）、异常（0xFF） |
| 档位  |   1   |                             当前标准档位                             |

### 黑加手环属性

| 名称  | 长度  |     描述     |
| :---: | :---: | :----------: |
| 步数  |   2   |   当天步数   |
| 心率  |   1   | 最后一次心率 |
| 状态  |   1   | 当前活动状态 |


