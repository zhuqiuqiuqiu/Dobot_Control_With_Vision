1.通过conda环境=labelimg，进行数据的标注
2.标注后的数据分别分为img 和 label
2.打开data_all ->data_run -> data_split.py进行yolo格式数据的拆分,得到dataset-imgges、labels、data.yaml
3.运行main.py函数 