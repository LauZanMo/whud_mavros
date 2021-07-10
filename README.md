# whud_mavros

​		whud_mavros是用于mavros功能包的插件，我们提供了脚本进行快速更新插件，在使用脚本whud_mavros_update.sh前，以下条件需要满足：

1. mavros需要源码编译，不能二进制安装。

2. 源码mavros需要放在～/Code/mavros_ws/src的路径下。

   若不满足上述条件，则需要根据自身情况对脚本进行修改方可运行。

   更新插件之后，可参考config目录下的配置文件进行配置，即可使用该插件。