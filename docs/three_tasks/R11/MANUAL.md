# R11 手工检查

```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R11 check
bash optimization_v2/three_tasks/advance.sh R11 open
xdg-open optimization_v2/three_tasks/results/R11/spray/material/preview.png
xdg-open optimization_v2/three_tasks/results/R11/inspection/material/preview.png
xdg-open optimization_v2/three_tasks/results/R11/mowing/material/preview.png
```

比较同目录下 original/preview.png 与 material/preview.png：位置、外轮廓、传感器安装位置应一致，颜色和面板不同。comparison.json 给出真实收帧与退出码，world.visual.json 给出非材质哈希。截图成功不等于自主任务通过。

重新生成并复测，必须先退出其他仿真：
```bash
bash optimization_v2/three_tasks/advance.sh R11 build
bash optimization_v2/three_tasks/advance.sh R11 test
bash optimization_v2/three_tasks/advance.sh R11 finalize
```

R11 build 会重建独立派生目录，不改原始车模。不要在正在录制时执行 build，否则资产绑定将失效。
