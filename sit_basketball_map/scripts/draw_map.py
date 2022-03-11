from map_canvas import MapCanvas


def get_blank_map():
    return MapCanvas(resolution=0.01,
                     width=15,
                     height=10)


start_area_width = 1
start_area_height = 1


def get_visual_map():
    m = get_blank_map()

    def go_start():
        m.draw_start(0.5, 1)

    def go_top_right():
        go_start()
        m.draw_move(14, 0)

    def draw_start_area():
        m.rect(start_area_width, start_area_height)
        hw, hh = start_area_width / 2, start_area_height / 2
        m.draw_move(hw,hh)
        m.circle(0.1,True)
        m.draw_move(-hw, -hh)

    m.line_width = 0.2

    # 确定地图起点与边框
    go_start()
    m.rect(14, 7.5)

    # 绘制传球边界线(中心圈)
    m.draw_move(2.5, 3.75)
    m.circle(2.5)

    # 绘制中线置球区
    m.draw_move(-2.5, 0)
    m.line_to(5, 0)

    # 绘制A方机器人起始位
    go_start()
    m.draw_move(1, -1)
    draw_start_area()

    # 绘制B方机器人起始位
    m.draw_move(0, 1 + 7.5)
    draw_start_area()

    # 绘制A方传球目标区
    go_start()
    m.draw_move(4.5, 0)
    m.rect(2.5, 0.75)

    # 绘制B方传球目标区
    m.draw_move(0, 7.5 - 0.75)
    m.rect(2.5, 0.75)

    # 绘制三分线置球区
    go_top_right()
    m.draw_move(-3, 3.75)
    m.circle(3.75)

    # 绘制三分线内置球区
    line_length = 6.34429
    distance = (3.75 ** 2 - (line_length / 2) ** 2) ** 0.5
    go_top_right()
    m.draw_move(-3, 3.75)
    m.draw_move(-distance, 0)
    m.draw_move(0, -line_length / 2)
    m.line_to(0, line_length)

    # 绘制投篮边界区
    go_top_right()
    m.draw_move(-1.575, 3.75)
    m.circle(2)

    # 绘制投篮目标位
    m.circle(0.5, True)

    return m
