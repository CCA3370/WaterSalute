# WaterSalute - X-Plane 12 Water Salute Plugin

X-Plane 12 插件，模拟飞机过水门仪式。两辆消防车驶向飞机并喷射水柱形成水拱门。

## 功能

- 通过菜单控制启动/停止
- 自动检查飞机在地面且速度 < 40 节
- 消防车根据飞机翼展自动定位
- 8x8 消防车物理转向模型

## 安装

将 `WaterSalute` 文件夹复制到 `X-Plane 12/Resources/plugins/`

## 使用

1. 飞机在地面低速滑行
2. 菜单：**Plugins → Water Salute → Start Water Salute**
3. 点击 **Stop** 结束仪式

## Datarefs

所有 dataref 为 float 数组，索引 0 = 左侧消防车，索引 1 = 右侧消防车。

| Dataref | 单位 | 范围 | 读写 | 说明 |
|---------|------|------|------|------|
| `watersalute/truck/front_steering_angle` | 度 | -45 ~ 45 | R/W | 前轮转向角（前两组转向架共用） |
| `watersalute/truck/rear_steering_angle` | 度 | -45 ~ 45 | R/W | 后轮转向角（反向转向） |
| `watersalute/truck/wheel_rotation_angle` | 度 | 0 ~ 360 | R | 车轮旋转角度（根据车速实时计算） |
| `watersalute/truck/cannon_pitch` | 度 | 0 ~ 90 | R/W | 水炮俯仰角 |
| `watersalute/truck/cannon_yaw` | 度 | -180 ~ 180 | R/W | 水炮偏航角 |
| `watersalute/truck/speed` | m/s | - | R | 车辆速度 |

### 转向逻辑

- **前轮角度**为主控制（±45度）
- **后轮角度**由前轮推导：`rear_angle = -front_angle * 0.4`
- **转向速率**由前轮角度和车速计算：`turning_rate = (speed * tan(front_angle)) / wheelbase`
- 车辆航向根据转向速率更新

## License

MIT