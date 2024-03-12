# 使用说明

- `tasks.json` 文件中有 2 个任务：
    - `buildEmbeddedTargets` 任务作为 `lanuch.json` 中的 Prelaunchtask，在调试前先行编译文件，更方便
    - `Download to STM` 任务用于烧录程序，使用方法为在终端输入 `openocd -c 烧录.hex文件`
