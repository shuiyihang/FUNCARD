idf开发使用版本release/v4.3
下载idf,
安装开发工具./install.sh
在esp-idf文件夹使用export.sh，初始化话开发工具环境
idf.py menuconfig配置选项
在工程目录下，idf.py build 构建工程


ESP-IDF 目录
项目目录
用户自定义组件目录（可选）中查找所有组件

idf.py set-target esp32-s2

git submodule的是增加删除：
增加：
git submodule add https://github.com/lvgl/lvgl.git components/lvgl
删除


idf.py create-project --path my_projects my_new_project


奇怪的是，我把log 从detail改为info，问题就没了，但是编码器使用不算是正常，v8版本上面

v7验证应该是正常的编码器使用。

log是不是需要开print

在v8分支上有一个stash