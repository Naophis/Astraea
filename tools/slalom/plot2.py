import numpy as np
import matplotlib.pyplot as plt
import math

from slalom3 import Slalom3
from matplotlib import gridspec

plot_row = 5
plot_col = 2


class Plot2:
    def exe(self, type, tgt_v, show, mode=0, K=1, list_K_y=[], offset={}, hf_cl=0):

        # fig = plt.figure(figsize=(5, 5), dpi=500)
        fig = plt.figure(dpi=200, tight_layout=True)
        spec = gridspec.GridSpec(ncols=2, nrows=1,
                                 width_ratios=[2, 1])
        trj = plt.subplot2grid((plot_row, plot_col), (0, 0), rowspan=4)
        trj.set(facecolor="dimgrey")
        # trj.set(facecolor="black")
        v = tgt_v
        rad = 54
        rad2 = 54
        n = 2
        tgt_ang = 90
        slip_gain = 250
        end_pos = {"x": 90, "y": 90}
        start_ang = 0

        tgt_ang1 = tgt_ang2 = 0
        if type == "normal":
            if hf_cl == 0:
                rad = 22
                rad2 = 22
                n = 2
                tgt_ang1 = 50
                tgt_ang2 = 40
                end_pos = {"x": 45, "y": 45}
            elif hf_cl == 1:
                rad = 48
                n = 2
                tgt_ang1 = 50
                tgt_ang2 = 40
                end_pos = {"x": 90, "y": 90}
            start_ang = 0
        elif type == "large":
            if hf_cl == 0:
                rad = 56.5
                n = 4
                tgt_ang = 90
                start_ang = 0
                end_pos = {"x": 90, "y": 90}
            elif hf_cl == 1:
                rad = 57.5
                n = 4
                tgt_ang = 90
                start_ang = 0
                end_pos = {"x": 180, "y": 180}
        elif type == "orval":
            if mode == 1:
                rad = 54
                n = 0
                tgt_ang1 = 180.0 * 1 / 3
                tgt_ang2 = 180.0 * 2 / 3
                tgt_ang3 = 180.0
                tgt_ang = 180
                end_pos = {"x": 90, "y": 90}
                start_ang = 0
            else:
                if hf_cl == 0:
                    rad = 45.750
                    n = 4
                    tgt_ang = 180.5
                    # tgt_ang = 180
                    end_pos = {"x": 0, "y": 180}
                    start_ang = 0
                elif hf_cl == 1:
                    rad = 55.25
                    n = 4
                    tgt_ang = 180
                    end_pos = {"x": 0, "y": 360}
                    start_ang = 0

        elif type == "dia45":
            end_pos = {"x": 90, "y": 45}
            tgt_ang1 = 45.0 * 1 / 3
            tgt_ang2 = 45.0 * 2 / 3
            tgt_ang = 45.5
            start_ang = 0

            if hf_cl == 0:
                rad = 50.5
                n = 4
                end_pos = {"x": 90, "y": 45}
            elif hf_cl == 1:
                rad = 64
                n = 4
                end_pos = {"x": 180, "y": 90}

        elif type == "dia135":
            start_ang = 0

            if hf_cl == 0:
                # rad = 46.0
                rad = 45.0
                n = 4
                tgt_ang = 135.125
                end_pos = {"x": 45, "y": 90}
            elif hf_cl == 1:
                rad = 45
                n = 4
                tgt_ang = 135
                end_pos = {"x": 90, "y": 180}

        elif type == "dia45_2":
            start_ang = 45
            if hf_cl == 0:
                rad = 70
                n = 4
                tgt_ang = 45
                end_pos = {"x": 90, "y": 45}
            elif hf_cl == 1:
                rad = 70
                n = 4
                tgt_ang = 45
                end_pos = {"x": 180, "y": 90}

        elif type == "dia135_2":
            start_ang = 45
            if hf_cl == 0:
                rad = 39.50
                n = 4
                tgt_ang = 135.5
                end_pos = {"x": -45, "y": 90}
            elif hf_cl == 1:
                rad = 43
                n = 4
                tgt_ang = 135
                end_pos = {"x": -90, "y": 180}
        elif type == "dia90":
            start_ang = 0
            if hf_cl == 0:
                rad = 40.0
                n = 4
                tgt_ang = 90
                end_pos = {"x": 90/math.sqrt(2), "y": 90/math.sqrt(2)}
            elif hf_cl == 1:
                rad = 45
                n = 4
                tgt_ang = 90
                end_pos = {"x": 0, "y": 180}

        res = {}
        sla = Slalom3(v, rad, rad2, n, tgt_ang1, tgt_ang2, end_pos,
                      slip_gain, type, K, list_K_y)
        if hf_cl == 0:
            sla.set_cell_size(90)
        elif hf_cl == 1:
            sla.set_cell_size(180)

        sla.calc_base_time()
        sla.calc_base_time2()

        res = sla.calc(start_ang)

        # sla.calc_offset_front()
        start_pos_x = [0, 0]
        start_pos_y = [0, 0]
        # offset = {}
        # offset["prev"] = 10
        # offset["after"] = 0
        res1 = sla.calc_offset_dist(start_pos_x, start_pos_y, type, offset)
        range = [-1000, 1000]

        wall_color = "red"
        wall_width = 6
        wall_alpha = 0.25
        sub_line_color = "silver"
        subline_width = 0.75
        subline_alpha = 0.5
        trj_width = 3
        trj_alpha = 1

        # 壁境界
        trj.plot(range, [45, 45], ls="-", c=wall_color,
                 lw=wall_width, alpha=wall_alpha)
        trj.plot(range, [-45, -45], ls="-", c=wall_color,
                 lw=wall_width, alpha=wall_alpha)
        trj.plot([-45, -45], range, ls="-", c=wall_color,
                 lw=wall_width, alpha=wall_alpha)
        trj.plot([45, 45], range, ls="-", c=wall_color,
                 lw=wall_width, alpha=wall_alpha)
        trj.plot(range, [135, 135], ls="-", c=wall_color,
                 lw=wall_width, alpha=wall_alpha)
        trj.plot([135, 135], range, ls="-", c=wall_color,
                 lw=wall_width, alpha=wall_alpha)
        trj.plot([225, 225], range, ls="-", c=wall_color,
                 lw=wall_width, alpha=wall_alpha)
        trj.plot(range, [225, 225], ls="-", c=wall_color,
                 lw=wall_width, alpha=wall_alpha)

        # 点線
        trj.plot(range, [0, 0], ls="--", c=sub_line_color,
                 lw=subline_width, alpha=subline_alpha)
        trj.plot(range, [90, 90], ls="--", c=sub_line_color,
                 lw=subline_width, alpha=subline_alpha)
        trj.plot(range, [-90, -90], ls="--", c=sub_line_color,
                 lw=subline_width, alpha=subline_alpha)

        trj.plot(range, [180, 180], ls="--", c=sub_line_color,
                 lw=subline_width, alpha=subline_alpha)

        trj.plot([0, 0], range, ls="--", c=sub_line_color,
                 lw=subline_width, alpha=subline_alpha)
        trj.plot([90, 90], range, ls="--", c=sub_line_color,
                 lw=subline_width, alpha=subline_alpha)

        trj.plot([180, 180], range, ls="--", c=sub_line_color,
                 lw=subline_width, alpha=subline_alpha)

        # 前距離
        trj.plot(res1["prev_path_x"], res1["prev_path_y"], ls="-",
                 color="coral", lw=trj_width, alpha=trj_alpha)
        # メイン
        trj.plot(res["x"] + res1["turn_offset_x"], res["y"] + res1["turn_offset_y"], color="yellow", lw=trj_width,
                 alpha=trj_alpha)
        # # 後距離
        trj.plot(res1["after_path_x2"], res1["after_path_y2"],
                 ls="-", color="coral", lw=trj_width, alpha=trj_alpha)
        plW = plt.subplot2grid((plot_row, plot_col), (4, 1), rowspan=1)
        plW.plot(res["alpha"])

        start_pos_x = [0, 0]
        start_pos_y = [0, 0]
        # start_pos_y = [-10, -10]
        res = sla.calc(start_ang)

        res2 = sla.calc_offset_dist(start_pos_x, start_pos_y, type, offset)

        trj.plot(res2["prev_path_x"], res2["prev_path_y"],
                 ls="--", color="cyan", lw=1, alpha=trj_alpha)
        trj.plot(res["x"] + res2["turn_offset_x"], res["y"] + res2["turn_offset_y"], color="blue", lw=1,
                 alpha=trj_alpha, ls="--")
        trj.plot(res2["after_path_x2"], res2["after_path_y2"],
                 ls="--", color="cyan", lw=1, alpha=trj_alpha)
        if res2["after_path_x2"][1] != end_pos["x"] or \
                res2["after_path_y2"][1] != end_pos["y"]:
            print(res2["after_path_x2"])
            print(res2["after_path_y2"])
        # plV = plt.subplot2grid((plot_row, plot_col), (1, 0), rowspan=plot_col)
        plV = plt.subplot2grid((plot_row, plot_col), (0, 1), rowspan=1)
        plV.plot(res["v"] * 1000)
        plVx = plt.subplot2grid((plot_row, plot_col), (1, 1), rowspan=1)
        plVx.plot(res["vx"] * 1000)
        plVy = plt.subplot2grid((plot_row, plot_col), (2, 1), rowspan=1)
        plVy.plot(res["vy"] * 1000)
        plW = plt.subplot2grid((plot_row, plot_col), (3, 1), rowspan=1)
        plW.plot(res["w"])
        plW.plot(res["w2"])
        # plBeta = plt.subplot2grid((plot_row, plot_col), (4, 1), rowspan=1)
        # plBeta.plot(res["beta"] * 180 / np.pi)
        # plVx.plot(res["vx"])
        print('{}:'.format(type))
        print('  v: {}'.format(sla.v))
        print('  ang: {}'.format(sla.base_ang))
        print('  pow_n: {}'.format(sla.pow_n))
        print('  rad: {}'.format(sla.rad))
        print('  time: {}'.format(sla.base_time))
        if type == "orval":
            print('  rad2: {}'.format(sla.rad))
        if type == "orval":
            print('  time2: {}'.format(sla.base_time))

        print('  front: {{ left: {}, right: {} }}'.format(
            res2["prev_dist"], res2["prev_dist"]))
        print('  back: {{ left: {}, right: {} }}'.format(
            res2["after_dist"], res2["after_dist"]))

        accY = plt.subplot2grid((plot_row, plot_col), (4, 0), rowspan=1)
        accY.plot(res["acc_y"] / 9.8)

        trj.set_aspect('1.0')
        plot_range = [-60, 240]
        trj.set_xlim(plot_range)
        trj.set_ylim(plot_range)
        # plt.xlim(plot_range)
        # plt.ylim(plot_range)

        if show:
            acc_y = np.abs(res["acc_y"]).max()
            plt.suptitle("{}[G]".format(acc_y / 9.8))
            plt.show()
