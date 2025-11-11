from PIL import Image, ImageDraw
import numpy as np
import json

# 地图尺寸
WIDTH, HEIGHT = 800, 600
img = Image.new('L', (WIDTH, HEIGHT), 0)  # 黑色背景（开放空间）
draw = ImageDraw.Draw(img)

# 前期墙体厚度
WALL_THICKNESS = 4

# 绘制房间
def draw_wall(x1, y1, x2, y2):
    """绘制加厚的墙体"""
    draw.rectangle([(x1, y1), (x2, y1 + WALL_THICKNESS)], fill=255)  # 上
    draw.rectangle([(x1, y2 - WALL_THICKNESS), (x2, y2)], fill=255)  # 下
    draw.rectangle([(x1, y1), (x1 + WALL_THICKNESS, y2)], fill=255)  # 左
    draw.rectangle([(x2 - WALL_THICKNESS, y1), (x2, y2)], fill=255)  # 右

# 外墙
# draw_wall(0, 0, WIDTH-1, HEIGHT-1)

# 房间布局
ROOMS = {
    "room0": (100, 30, 450, 230), 
    "room1": (470, 30, 700, 230),   
    "room2": (30, 300, 250, 570),
    "room3": (270, 300, 470, 570),
    "room4": (490, 300, 770, 570)
}

# 绘制房间
for room, (x1, y1, x2, y2) in ROOMS.items():
    draw_wall(x1, y1, x2, y2)
    
print(ROOMS)

# 绘制门
draw.rectangle([(263, 226), (287, 230)], fill=0) # room0---door
draw.rectangle([(573, 226), (597, 230)], fill=0) # room1---door
draw.rectangle([(128, 300), (152, 304)], fill=0) # room2---door
draw.rectangle([(358, 300), (382, 304)], fill=0) # room3---door
draw.rectangle([(618, 300), (642, 304)], fill=0) # room4---door

# 绘制房间内物体
# room0---客厅
# 沙发 (L形)
draw.rectangle([(120, 50), (190, 80)], fill=255)  # 沙发主体

# 茶几 (沙发前)
draw.rectangle([(140, 90), (160, 110)], fill=255)

# 茶杯 (紧挨茶几)
draw.rectangle([(150, 100), (155, 105)], fill=255)

# 桌子 (沙发旁边)
draw.rectangle([(210, 50), (230, 70)], fill=255)

# 电视柜
draw.rectangle([(110, 150), (190, 160)], fill=255)

# 电视机 (紧挨电视柜)
draw.rectangle([(120, 140), (180, 150)], fill=255)

# 书柜
draw.rectangle([(130, 170), (150, 220)], fill=255)

# 书 (紧挨书柜)
draw.rectangle([(135, 175), (145, 180)], fill=255)  # 书1
draw.rectangle([(135, 185), (145, 190)], fill=255)  # 书2

# ==========
# 餐桌
draw.rectangle([(280, 100), (350, 120)], fill=255)

# 餐座椅 (环绕餐桌)
draw.rectangle([(270, 90), (280, 100)], fill=255)  # 左上
draw.rectangle([(350, 90), (360, 100)], fill=255)   # 右上
draw.rectangle([(270, 120), (280, 130)], fill=255)  # 左下
draw.rectangle([(350, 120), (360, 130)], fill=255)  # 右下

# 盘子 (紧挨餐桌)
draw.rectangle([(300, 110), (310, 115)], fill=255)

# 杯子 (紧挨盘子)
draw.rectangle([(315, 110), (320, 115)], fill=255)

# 叉子 (紧挨盘子)
draw.rectangle([(305, 116), (310, 118)], fill=255)

# ==========
# 花瓶
draw.rectangle([(380, 50), (385, 60)], fill=255)

# 镜子
draw.rectangle([(390, 70), (420, 90)], fill=255)

# 垃圾桶
draw.rectangle([(380, 150), (390, 160)], fill=255)

# 衣帽架
draw.rectangle([(400, 180), (405, 200)], fill=255)  # 支架
draw.rectangle([(395, 200), (410, 205)], fill=255) # 横杆

# 大衣 (紧挨衣帽架)
draw.rectangle([(395, 185), (410, 195)], fill=255)

# 帽子 (紧挨衣帽架)
draw.rectangle([(407, 175), (412, 180)], fill=255)

# 饮水机
draw.rectangle([(420, 150), (440, 170)], fill=255)

# 水杯 (紧挨饮水机)
draw.rectangle([(430, 140), (435, 145)], fill=255)



# room1---中型卧室
# 衣柜
draw.rectangle([(480, 60), (520, 180)], fill=255)

# 衣服 (在衣柜里)
draw.rectangle([(485, 50), (515, 70)], fill=255)  # 上衣
draw.rectangle([(485, 80), (515, 100)], fill=255)  # 裤子
draw.rectangle([(485, 110), (515, 130)], fill=255) # 外套

# 梳妆台 (衣柜下方)
draw.rectangle([(480, 210), (520, 220)], fill=255)

# 椅子 (梳妆台右边)
draw.rectangle([(530, 210), (540, 220)], fill=255)

# 镜子 (梳妆台上)
draw.rectangle([(490, 200), (510, 210)], fill=255)

# 梳子 (梳妆台上)
draw.rectangle([(495, 205), (500, 208)], fill=255)

# 台灯 (梳妆台上)
draw.rectangle([(505, 205), (510, 210)], fill=255)

# ==========
# 床
draw.rectangle([(570, 80), (670, 180)], fill=255)

# 枕头 (床上)
draw.rectangle([(570, 90), (590, 110)], fill=255)

# 被子 (床上)
draw.rectangle([(560, 120), (680, 170)], fill=255)

# 左边床头柜
draw.rectangle([(540, 100), (550, 120)], fill=255)

# 夜灯 (左边床头柜上)
draw.rectangle([(545, 95), (548, 100)], fill=255)

# 闹钟 (左边床头柜上)
draw.rectangle([(545, 122), (550, 125)], fill=255)

# 右边床头柜
draw.rectangle([(680, 100), (690, 120)], fill=255)

# 夜灯 (右边床头柜上)
draw.rectangle([(695, 95), (698, 100)], fill=255)

# 体重秤 (床下边)
draw.rectangle([(625, 185), (645, 190)], fill=255)

# 空调 (床和体重秤下边)
draw.rectangle([(620, 210), (650, 220)], fill=255)



# room2---厨房
# 橱柜 (L形)
draw.rectangle([(40, 320), (100, 350)], fill=255)  # 水平部分
draw.rectangle([(40, 350), (60, 450)], fill=255)   # 垂直部分

# 灶台 (在橱柜前面)
draw.rectangle([(70, 360), (90, 380)], fill=255)

# 水龙头 (紧挨灶台)
draw.rectangle([(85, 360), (95, 365)], fill=255)

# 抽油烟机 (紧挨灶台上方)
draw.rectangle([(65, 340), (95, 350)], fill=255)

# 微波炉 (紧挨灶台右侧)
draw.rectangle([(100, 360), (120, 380)], fill=255)

# 烤箱 (紧挨微波炉下方)
draw.rectangle([(70, 390), (90, 410)], fill=255)

# 洗碗机 (紧挨烤箱下方)
draw.rectangle([(70, 420), (90, 450)], fill=255)

# 碗 (紧挨洗碗机)
draw.rectangle([(80, 430), (85, 435)], fill=255)

# 冰箱 (在灶台前面)
draw.rectangle([(40, 460), (80, 540)], fill=255)

# 苹果 (紧挨冰箱)
draw.rectangle([(85, 470), (90, 475)], fill=255)

# 香蕉 (紧挨苹果)
draw.rectangle([(85, 480), (90, 485)], fill=255)

# 蔬菜类 (紧挨香蕉)
draw.rectangle([(85, 490), (90, 500)], fill=255)

# ==========
# 桌子 (中央操作台)
draw.rectangle([(150, 400), (230, 430)], fill=255)  # 原(150,350)-(230,380)

# 砧板 (紧挨桌子) - y坐标+50
draw.rectangle([(160, 435), (180, 445)], fill=255)  # 原(160,385)-(180,395)

# 菜刀 (紧挨砧板) - y坐标+50
draw.rectangle([(185, 435), (190, 445)], fill=255)  # 原(185,385)-(190,395)

# 锅具 (紧挨桌子) - y坐标+50
draw.rectangle([(200, 435), (220, 450)], fill=255)  # 原(200,385)-(220,400)

# 电子秤 (紧挨桌子) - y坐标+50
draw.rectangle([(155, 390), (165, 400)], fill=255)  # 原(155,340)-(165,350)

# 勺子 (紧挨电子秤) - y坐标+50
draw.rectangle([(170, 390), (175, 395)], fill=255)  # 原(170,340)-(175,345)

# 筷子 (紧挨勺子) - y坐标+50
draw.rectangle([(180, 390), (185, 395)], fill=255)  # 原(180,340)-(185,345)

# 餐盘 (紧挨桌子) - y坐标+50
draw.rectangle([(160, 440), (170, 450)], fill=255)  # 原(160,390)-(170,400)

# 碗 (紧挨餐盘) - y坐标+50
draw.rectangle([(175, 440), (180, 445)], fill=255)  # 原(175,390)-(180,395)

# 杯子 (紧挨碗) - y坐标+50
draw.rectangle([(185, 440), (190, 445)], fill=255)  # 原(185,390)-(190,395)

# 垃圾桶 (桌子前面) - y坐标+50
draw.rectangle([(180, 460), (200, 480)], fill=255)  # 原(180,410)-(200,430)

# 咖啡机 (桌子旁边) - y坐标+50
draw.rectangle([(140, 450), (160, 470)], fill=255)  # 原(140,400)-(160,420)



# room3---卫生间
# 花洒 - y坐标+50
draw.rectangle([(280, 360), (290, 370)], fill=255)  # 原(280,310)-(290,320)

# 洗浴镜 - y坐标+50
draw.rectangle([(300, 360), (330, 390)], fill=255)  # 原(300,310)-(330,340)

# 香皂 (紧挨洗浴镜) - y坐标+50
draw.rectangle([(335, 365), (340, 370)], fill=255)  # 原(335,315)-(340,320)

# 洗发水 (紧挨洗浴镜) - y坐标+50
draw.rectangle([(335, 375), (340, 380)], fill=255)  # 原(335,325)-(340,330)

# 洗浴柜 - y坐标+50
draw.rectangle([(280, 400), (320, 420)], fill=255)  # 原(280,350)-(320,370)

# 洗面奶 (在洗浴柜里) - y坐标+50
draw.rectangle([(285, 405), (290, 410)], fill=255)  # 原(285,355)-(290,360)

# 牙刷牙膏 (在洗浴柜里) - y坐标+50
draw.rectangle([(295, 405), (300, 410)], fill=255)  # 原(295,355)-(300,360)

# 洗衣机 (右边) - y坐标+50
draw.rectangle([(420, 360), (450, 410)], fill=255)  # 原(350,310)-(380,360)

# 脏衣服 (紧挨洗衣机) - y坐标+50
draw.rectangle([(420, 420), (450, 430)], fill=255)  # 原(385,320)-(395,340)

# ===== 下部分(马桶区) =====
# 马桶
draw.rectangle([(340, 500), (380, 550)], fill=255)

# 消毒液 (紧挨马桶)
draw.rectangle([(335, 490), (340, 495)], fill=255)

# 厕纸 (紧挨马桶)
draw.rectangle([(385, 490), (390, 495)], fill=255)



# room4---大型卧室
# 衣柜
draw.rectangle([(520, 310), (560, 450)], fill=255)  # 原(500,310)-(540,450)

# 试衣镜 (衣柜旁边)
draw.rectangle([(565, 310), (590, 350)], fill=255)  # 原(545,310)-(570,350)

# 衣服 (在衣柜里)
draw.rectangle([(525, 320), (555, 340)], fill=255)  # 上衣 原(505,320)-(535,340)
draw.rectangle([(525, 350), (555, 370)], fill=255)  # 裤子 原(505,350)-(535,370)
draw.rectangle([(525, 380), (555, 400)], fill=255) # 外套 原(505,380)-(535,400)

# 梳妆台 (衣柜下方)
draw.rectangle([(520, 460), (560, 480)], fill=255)  # 原(500,460)-(540,480)

# 台灯 (紧挨梳妆台)
draw.rectangle([(565, 460), (575, 470)], fill=255)  # 原(545,460)-(555,470)

# ==========
# 空调
draw.rectangle([(680, 310), (740, 330)], fill=255)  # 原(700,310)-(760,330)

# 书架
draw.rectangle([(680, 350), (740, 450)], fill=255)  # 原(700,350)-(760,450)

# 书 (在书架上)
draw.rectangle([(685, 355), (695, 365)], fill=255)  # 书1 原(705,355)-(715,365)
draw.rectangle([(700, 355), (710, 365)], fill=255)  # 书2 原(720,355)-(730,365)
draw.rectangle([(715, 355), (725, 365)], fill=255)  # 书3 原(735,355)-(745,365)

# 电视柜
draw.rectangle([(680, 460), (740, 480)], fill=255)  # 原(700,460)-(760,480)

# 电视 (电视柜上)
draw.rectangle([(690, 440), (730, 460)], fill=255)  # 原(710,440)-(750,460)

# 电视遥控器 (电视柜上)
draw.rectangle([(700, 485), (710, 490)], fill=255)  # 原(720,485)-(730,490)

# 空调遥控器 (电视柜上)
draw.rectangle([(720, 485), (730, 490)], fill=255)  # 原(740,485)-(750,490)

# =========
# 床
draw.rectangle([(530, 490), (650, 560)], fill=255)

# 枕头 (床上)
draw.rectangle([(560, 500), (580, 530)], fill=255)

# 被子 (床上)
draw.rectangle([(540, 540), (660, 550)], fill=255)

# 抱枕 (床上)
draw.rectangle([(560, 530), (570, 550)], fill=255)

# 左边床头柜
draw.rectangle([(500, 500), (520, 560)], fill=255)

# 夜灯 (左边床头柜上)
draw.rectangle([(505, 495), (510, 500)], fill=255)

# 闹钟 (左边床头柜上)
draw.rectangle([(505, 522), (510, 525)], fill=255)




# 对除房间和走廊外的区域进行填充，无物理意义
draw.rectangle([(0, 0), (100, 230)], fill=255)
draw.rectangle([(100, 0), (700, 30)], fill=255)
draw.rectangle([(450, 30), (470, 230)], fill=255)
draw.rectangle([(700, 0), (800, 230)], fill=255)
draw.rectangle([(0, 300), (30, 570)], fill=255)
draw.rectangle([(250, 300), (270, 570)], fill=255)
draw.rectangle([(470, 300), (490, 570)], fill=255)
draw.rectangle([(770, 300), (800, 570)], fill=255)
draw.rectangle([(0, 570), (800, 600)], fill=255)
draw.rectangle([(0, 230), (4, 300)], fill=255)
draw.rectangle([(796, 230), (800, 300)], fill=255)


# img.save('/home/wyc/LiMRos_ws/src/dmce-master/dmce_sim/maps/apartment1.png')
meta = {"width": WIDTH, "height": HEIGHT, "resolution": 0.04, "rooms": ROOMS}
with open("apartment1_rooms.json", "w") as f:
    json.dump(meta, f, indent=2)

# 验证图像只有纯黑白
img_arr = np.array(img)
unique_vals = np.unique(img_arr)
if list(unique_vals) == [0, 255]:
    print("✓ 验证成功: 只有纯黑(0)和纯白(255)")
else:
    print(f"! 发现异常值: {unique_vals}")