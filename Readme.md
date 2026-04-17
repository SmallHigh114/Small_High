# 606算法主仓库

## 仓库结构

- **lidar**
  - `lidar-fix` - 雷达保护主分支
  - `lidar` - 雷达主分支
  - `lidar-dev` - 开发中雷达分支

- **navigation**
  - `navigation-fix` - 导航保护主分支
  - `navigation` - 导航主分支
  - `navigation-dev` - 开发中导航主分支

- **vision-infantry**（步兵自瞄）
  - `vision-infantry-fix` - 步兵自瞄保护主分支
  - `vision-infantry` - 步兵自瞄主分支
  - `vision-infantry-dev` - 开发中步兵自瞄主分支

- **vision-guard**（哨兵自瞄）
  - `vision-guard-fix` - 哨兵自瞄保护主分支
  - `vision-guard` - 哨兵自瞄主分支
  - `vision-guard-dev` - 开发中哨兵自瞄主分支

- **vision-hero**（英雄自瞄）
  - `vision-hero-fix` - 英雄自瞄保护主分支
  - `vision-hero` - 英雄自瞄主分支
  - `vision-hero-dev` - 开发中英雄自瞄主分支

- **vision-UAV**（无人机自瞄）
  - `vision-UAV-fix` - 无人机自瞄保护主分支
  - `vision-UAV` - 无人机自瞄主分支
  - `vision-UAV-dev` - 开发中无人机自瞄主分支

- **vision-dart**（飞镖自瞄）
  - `vision-dart-fix` - 飞镖自瞄保护主分支
  - `vision-dart` - 飞镖自瞄主分支
  - `vision-dart-dev` - 开发中飞镖自瞄主分支

- **vision-rune**（能量机关）
  - `vision-rune-fix` - 能量机关保护主分支
  - `vision-rune` - 能量机关主分支
  - `vision-rune-dev` - 开发中能量机关主分支

---

## 提交要求

### 分支提交规则
- **dev分支**：可自行提交
- **主分支**：代码需经过验收后方可提交
- **fix分支**：仅战队特定阶段可验收提交：
  - 赛季初期
  - 中期考核
  - 完整形态
  - 超对封车

### Commit 规范
- 提交需清晰写明变动情况

### 禁止事项
- 禁止在 `master` 分支提交源码
- 禁止提交编译生成文件
- 禁止提交源码中包含 **20MB 以上** 的测试视频

### 提交频率
- 周提交次数不得少于 **两次**：**周二**、**周六**