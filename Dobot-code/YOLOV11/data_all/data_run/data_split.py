import os
import random
import shutil
import yaml

image_dir = r"E:\Dobot-code\YOLOV11\data_all\img"
label_dir = r"E:\Dobot-code\YOLOV11\data_all\label"

output_dir = "../../dataset"
train_ratio = 0.8

# ====== ⭐ 你必须改这里：类别 ======
classes = ["GPS", "WIFI","Square_red", "Square_yellow","Square_silver", "CHIP"]   # 改成你的类别

# ===================================

images = [f for f in os.listdir(image_dir) if f.endswith(('.jpg', '.png'))]

random.shuffle(images)

train_size = int(len(images) * train_ratio)

train_imgs = images[:train_size]
val_imgs = images[train_size:]


def copy_data(img_list, split):
    img_out = os.path.join(output_dir, "images", split)
    label_out = os.path.join(output_dir, "labels", split)

    os.makedirs(img_out, exist_ok=True)
    os.makedirs(label_out, exist_ok=True)

    for img in img_list:
        name = os.path.splitext(img)[0]

        # 图片
        shutil.copy(os.path.join(image_dir, img),
                    os.path.join(img_out, img))

        # 标签
        label_file = name + ".txt"
        label_path = os.path.join(label_dir, label_file)

        if os.path.exists(label_path):
            shutil.copy(label_path,
                        os.path.join(label_out, label_file))


# ====== 执行划分 ======
copy_data(train_imgs, "train")
copy_data(val_imgs, "val")


# ====== ⭐ 自动生成 data.yaml ======
data_yaml = {
    "path": os.path.abspath(output_dir),   # 自动转绝对路径
    "train": "images/train",
    "val": "images/val",
    "names": {i: name for i, name in enumerate(classes)}
}

yaml_path = os.path.join(output_dir, "data.yaml")

with open(yaml_path, "w", encoding="utf-8") as f:
    yaml.dump(data_yaml, f, sort_keys=False, allow_unicode=True)


print("划分完成！")
print("train:", len(train_imgs))
print("val:", len(val_imgs))
print("data.yaml 已生成：", yaml_path)