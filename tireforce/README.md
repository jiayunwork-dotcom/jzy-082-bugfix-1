# tireforce — 轮胎纵向力 HTTP 服务

经典经验轮胎纵向力模型（Pacejka "Magic Formula"，纯纵向滑移、无偏移量）的固化实现，
以 HTTP/JSON 服务形式提供，供整车纵向牵引/制动仿真主循环高频调用。

模型公式：

```
x  = B · κ
y  = x − E · (x − atan(x))        # 滑移率刚度/曲率修正（internal/tire/slip.go）
Fx = D · sin(C · atan(y))         # 主曲线求值（internal/tire/formula.go）
D  = peak（直接给定）或 friction · Fz（摩擦系数换算，唯一定义于 internal/tire/peak.go）
```

## 构建与运行（容器）

```bash
docker build -t tireforce .
docker run --rm -p 8080:8080 tireforce
```

运行时基础镜像为 `golang:1.22`，构建阶段会先跑完整测试套件再编译。
监听端口默认 `8080`，可用环境变量 `PORT` 覆盖。

本地开发（Go 1.22）：

```bash
go test ./...
go build -o tireforce ./cmd/server && ./tireforce
```

## 接口

所有接口均为 JSON 交互。系数对象 `coefficients` 字段：

| 字段        | 含义                     |
|-------------|--------------------------|
| `stiffness` | 刚度因子 B               |
| `shape`     | 形状因子 C               |
| `curvature` | 曲率因子 E               |
| `peak`      | 峰值幅度 D（直接给定）   |
| `friction`  | 摩擦系数 μ（D = μ·Fz）   |

`peak` 与 `friction` 必须且只能提供一个。

### POST /api/v1/longitudinal/force — 单点求值

```bash
curl -X POST localhost:8080/api/v1/longitudinal/force -d '{
  "vertical_load": 4000, "slip": 0.05,
  "coefficients": {"stiffness": 10, "shape": 1.9, "curvature": 0.97, "friction": 1.0}
}'
# => {"slip":0.05,"longitudinal_force":2942.47...,"peak_amplitude":4000}
```

返回纵向力与实际使用的峰值幅度；|κ| > 1 时附带 `warnings` 但仍正常计算。

### POST /api/v1/longitudinal/peak — 扫描寻峰

```bash
curl -X POST localhost:8080/api/v1/longitudinal/peak -d '{
  "vertical_load": 4000,
  "coefficients": {"stiffness": 10, "shape": 1.9, "curvature": 0.97, "friction": 1.0},
  "sweep": {"min": 0, "max": 1, "points": 2000}
}'
# => {"peak_slip":0.1801...,"peak_force":4000,"peak_amplitude":4000,"sweep":{...}}
```

`sweep` 可省略，默认 `{"min":0,"max":1,"points":2000}`。扫描与单点共用同一公式与同一
峰值幅度换算：对返回的 `peak_slip` 调单点接口复算，结果与 `peak_force` 完全一致
（有测试锁定）。扫描中途失败时返回 500 `sweep_failed`，绝不返回不完整结果。

### POST /api/v1/longitudinal/curve — 曲线采样

```bash
curl -X POST localhost:8080/api/v1/longitudinal/curve -d '{
  "vertical_load": 4000,
  "coefficients": {"stiffness": 10, "shape": 1.9, "curvature": 0.97, "friction": 1.0},
  "slips": [-0.2, -0.1, 0, 0.1, 0.18, 0.3]
}'
# => {"peak_amplitude":4000,"points":[{"slip":-0.2,"force":-3996.7...}, ...]}
```

### GET /api/v1/demo — 内置乘用车示范参数自检

返回示范参数（B=10, C=1.9, E=0.97, μ=1.0, Fz=4000 N）及 κ=0.05 处的采样结果：
小滑移率下力与滑移率同号、且低于峰值幅度。

### GET /healthz — 健康检查

## 错误与告警

输入合法性在任何计算之前完成校验，结构化错误响应：

```json
{"error": {"type": "invalid_load",  "field": "vertical_load", "message": "..."}}
{"error": {"type": "invalid_shape", "field": "shape",         "message": "..."}}
{"error": {"type": "invalid_sweep", "field": "sweep.points",  "message": "..."}}
```

- `invalid_load`（HTTP 400）：垂直载荷非正或非有限值；
- `invalid_shape`（HTTP 400）：形状因子 C ≤ 0、刚度 B = 0、系数非有限值、
  峰值幅度缺失/冲突/非正；
- `invalid_sweep`（HTTP 400）：扫描区间或点数非法；
- `sweep_failed`（HTTP 500）：扫描中途失败，不携带任何部分结果。

## 并发与隔离

模型包为纯函数实现，无任何共享可变状态；每个请求的系数、扫描中间量都保存在
请求级局部变量中，多路仿真客户端并发调用互不影响（有并发测试锁定）。

## 工程结构

```
cmd/server/main.go            入口（PORT 环境变量，默认 8080）
internal/tire/slip.go         滑移率刚度/曲率修正
internal/tire/formula.go      主曲线公式求值（单点）
internal/tire/peak.go         峰值幅度 D ⇔ 垂直载荷换算（唯一定义）
internal/tire/sweep.go        滑移区间扫描寻峰 + 曲线采样
internal/tire/validate.go     参数合法性校验
internal/tire/demo.go         内置乘用车示范参数
internal/api/router.go        HTTP 接口层（Gin）
```

## 测试锁定的物理性质

- 零滑移 → 纵向力恰为零；
- 滑移率取反 → 力大小不变、方向相反（无水平偏移）；
- D = μ·Fz 设定下载荷放大 k 倍 → 峰值力同比例放大 k 倍；
- 标准形状下任意工况 |Fx| ≤ D（含数值容差）；
- 扫描峰值点用单点公式复算 → 与扫描报告的峰值力完全一致；
- 峰值幅度换算单点定义：单点、扫描、曲线三条路径共用同一 D。
