import numpy as np
import cv2 as cv
import math
# from toolset import make_border_mask

def pt2pt_2d_range(pt1,pt2):
    return math.sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2)



def cover_pt_by_area(pt_xy,area_w_h = [1200,1200],limit_box = [0,0,4504,4504]):
    area_x1 = max(limit_box[0],int(pt_xy[0]-area_w_h[0]/2))
    area_x2 = area_x1+area_w_h[0]
    if area_x2>=limit_box[2]:
        area_x2 = limit_box[2]-1
        area_x1 = max(area_x2-area_w_h[0],limit_box[0])
    area_y1 = max(limit_box[1], int(pt_xy[1] - area_w_h[1] / 2))
    area_y2 = area_y1 + area_w_h[1]
    if area_y2 >= limit_box[3]:
        area_y2 = limit_box[3] - 1
        area_y1 = max(area_y2 - area_w_h[1], limit_box[1])
    return [area_x1,area_y1,area_x2,area_y2]




def calc_panoram_size(sect_da,observ_a,frame_size):
    return int(frame_size*sect_da/observ_a)


def calc_scan_areas(scan_area_bbox,window_w_h = (800, 800),overlay=(0.1,0.1),force_int = True):
    '''
    Вычисление подобластей заданного размера (окон) для сканирования в заданной области
    с перекрытием не менее указанного в параметре overlay
    :param bbox: область сканирования
    :param window_w_h: (ширина, высота) окна сканирования
    :param overlay: коэффициенты перекрытия (overlay_x,overlay_y) по соответствующим осям
    :return: массив bbox_w окон
    '''
    # scan_area_w = scan_area_bbox[2] - scan_area_bbox[0]
    # scan_area_h = scan_area_bbox[3] - scan_area_bbox[1]
    # print(f'area w,h = {scan_area_w},{scan_area_h}')
    # over_px_x = overlay[0]*window_w_h[0]
    # print(f' over x = {over_px_x}')
    #
    # n_x = (scan_area_w - window_w_h[0]) / (window_w_h[0] - over_px_x)
    # print(f' n x = {n_x}')
    # n_x = math.ceil(n_x)
    # over_px_x = window_w_h[0] - (scan_area_w-window_w_h[0])/n_x
    # print(f' n_x = {n_x}, over_px_x = {over_px_x}')
    #
    #
    # over_px_y = overlay[1]*window_w_h[1]
    x_areas = calc_linear_scan_areas((scan_area_bbox[0],scan_area_bbox[2]),window_w_h[0],overlay[0],force_int)
    y_areas = calc_linear_scan_areas((scan_area_bbox[1],scan_area_bbox[3]),window_w_h[1],overlay[1],force_int)

    areas = []
    for y_a in y_areas:
        for x_a in x_areas:
            a_bbox = [x_a[0], y_a[0],x_a[1],y_a[1]]
            areas.append(a_bbox)
    # print('boxes')
    # print(areas)
    return areas


def calc_linear_scan_areas(scan_area: tuple, window_w=800, overlay=0.1, force_int = True):
    '''
    Вычисление подобластей заданного размера (окон) для сканирования в заданной области
    с перекрытием не менее указанного в параметре overlay вдоль оси
    :param scan_area: область сканирования x1<->x2
    :param window: размер окна сканирования
    :param overlay: коэффициенты перекрытия
    :return:
    '''
    # print("I'm alive")
    scan_area_w = scan_area[1]-scan_area[0]
    over = overlay * window_w
    # print(f' over= {over}')

    n_x = (scan_area_w - window_w) / (window_w - over)
    # print(f' n = {n_x}')
    n_x = math.ceil(n_x)
    over = window_w - (scan_area_w - window_w) / n_x
    # print(f' n_x = {n_x}, over_px_x = {over}')
    window_arr = []
    for i in range(n_x+1):
        start_x = scan_area[0]+i*(window_w-over)
        stop_x = start_x+window_w
        if force_int:
            window_arr.append((int(start_x),int(stop_x)))
        else:
            window_arr.append((start_x,stop_x))
    return window_arr

def calc_scan_points(sect_a1,sect_da, observ_a, overlay = 1/3, orient = 'h'):
    '''
    Вычисление углов поворота для сканирования заданной области с перекрытием не менее указанного в параметре overlay
    :param sect_a1: начальный угол
    :param sect_da: ширина сектора сканирования
    :param observ_a: угол обзора
    :param overlay: коэффициент перекрытия
    :return: массив абсолютных значений углов для установки камеры
    '''
    over = observ_a*overlay
    n = (sect_da-observ_a)/(observ_a-over)

    print(f'n = {n}, ceil(n) = {math.ceil(n)}')
    n = math.ceil(n)
    over = observ_a - (sect_da-observ_a)/n

    print(f'over = {over}')
    angles = [0.0]*(n+1)
    for i,a in enumerate(angles):
        print(i)
        if orient == 'h':
            angles[i] = (sect_a1+ observ_a/2+i*(observ_a-over))%360
        else:
            angles[i] = (sect_a1 + observ_a / 2 + i * (observ_a - over))
    print(angles)

    # shift_a = (1-overlay)*observ_a
    # no_overlay_a = (1-overlay)*observ_a
    # n_steps_norm = math.ceil(sect_da/shift_a)
    # shift_norm = sect_da/n_steps_norm
    # over_norm = sect_da-shift_norm
    # print(f'Сдвиг идеальный: {shift_a}, нормированный: {shift_norm}')
    # angles = [observ_a/2]*(n_steps_norm)

    # for i in range(n_steps_norm):
    #     angles[i]+=(i*(sect_da-over_norm))
    # # print(angles)
    # return angles
    return angles

def calc_scan_points_a1_a2(sect_a1,sect_a2,observ_a,overlay = 1/3,orient = 'h'):
    '''
    :param sect_a1: начальный угол
    :param sect_a2: конечный угол сектора сканирования
    :param observ_a: угол обзора
    :param overlay: коэффициент перекрытия
    :return: массив абсолютных значений углов для установки камеры
    '''
    if orient == 'h':
        a_c, d_a = calc_sector(sect_a1,sect_a2)
    else:
        d_a = sect_a2-sect_a1
    return calc_scan_points(sect_a1,d_a,observ_a,overlay,orient)


def calc_sector(a1,a2):
    '''Вычисляет центр сектора и его ширину. Сектор задан от a1 к a2 по часовой стрелке
    :return: a_center, d_a - середина сектора, ширина сектора
    '''
    da = (a2-a1)%360
    return (a1+da/2)%360,da

def check_in_sector(a,a0,d_a):
    '''проверяет, находится ли угол [a] внутри сектора,
    заданного центром [a0] и шириной [d_a]'''
    a_ref = (a-a0)%360
    if a_ref>=(-d_a/2) and a_ref<=(d_a/2):
        return True
    else:
        return False

def check_in_range(x,range:()):
    if (x>=range[0]) and (x<=range[1]):
        return True
    else:
        return False

def linear_cross(obj_1,obj_2):
    cross = 0
    if check_in_range(obj_1[0],(obj_2[0],obj_2[1])):
        cross = min(obj_1[1],obj_2[1])-obj_1[0]
    elif check_in_range(obj_2[0],(obj_1[0],obj_1[1])):
        cross = min(obj_1[1],obj_2[1])-obj_2[0]
    return cross

def bbox_cross_area(bbox1,bbox2):
    x_cross = linear_cross((bbox1[0],bbox1[2]),(bbox2[0],bbox2[2]))
    y_cross = linear_cross((bbox1[1],bbox1[3]),(bbox2[1],bbox2[3]))
    cross_area = x_cross*y_cross
    return cross_area


