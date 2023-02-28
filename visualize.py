#!/usr/bin/env python3
import yaml
import matplotlib
# matplotlib.use("Agg")
from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import matplotlib.animation as manimation
import argparse
import math
from ast import literal_eval as make_tuple
import json
scale = 3
Colors = ['orange']  # , 'blue', 'green']


def color_for_goal(g, m, n):
    start_color = [1, 0, 0]
    start_color[1] = start_color[1]+g[0]/m
    start_color[2] = start_color[2]+g[1]/n
    start_color[0] = start_color[0]-(g[0]+g[1])/(2.5*m)

    return np.array(start_color)


class Animation:
    def __init__(self, map_sizeX, map_sizeY, paths, obstacles=[]):
        self.map_sizeX = map_sizeX
        self.map_sizeY = map_sizeY
        # self.schedule = schedule
        self.paths = paths
        self.obstacles = obstacles
        # aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]
        aspect = 1
        sizex = 16
        self.fig = plt.figure(frameon=False, figsize=(sizex * aspect, sizex))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(
            left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)
        # self.ax.set_frame_on(False)

        self.patches = []
        self.artists = []
        self.agents = dict()
        self.agent_names = dict()
        # create boundary patch
        xmin = -0.5
        ymin = -0.5
        xmax = map_sizeX - 0.5
        ymax = map_sizeY - 0.5

        # self.ax.relim()
        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)
        # self.ax.set_xticks([])
        # self.ax.set_yticks([])
        # plt.axis('off')
        # self.ax.axis('tight')
        # self.ax.axis('off')
        print("starting!")

        self.patches.append(Rectangle(
            (xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red'))
        # self.generate_obs()
        for o in self.obstacles:
            x, y = o[0], o[1]
            self.patches.append(
                Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='black', edgecolor='red'))
        self.arrange_random_colors(map_sizeX, map_sizeY)
        # self.arrange_random_colors(5, 5)
        # self.arrange_column_colors()
        # self.colors = np.random.rand(len(map["agents"]),3)
        # create agents:
        self.T = 0
        # draw goals first
        for i in range(0, len(paths)):
            goali = self.paths[i][-1]
            # self.patches.append(Rectangle((goali[0] - 0.25, goali[1] - 0.25), 0.5, 0.5,
            #                     facecolor=self.colors[i % len(self.colors)], edgecolor='black', alpha=0.15))
            # self.patches.append(Rectangle((d["goal"][0] - 0.25, d["goal"][1] - 0.25), 0.5, 0.5, facecolor=self.colors[i%len(self.colors)], edgecolor='black', alpha=0.15))
        for i in range(0, len(paths)):
            # name = d["name"]
            starti = self.paths[i][0]
            self.agents[i] = Circle((starti[0], starti[1]), 0.3, facecolor=self.colors[i % len(
                self.colors)], edgecolor='black')
            self.agents[i].original_face_color = self.colors[i %
                                                            len(self.colors)]
            self.patches.append(self.agents[i])
            self.T = max(self.T, len(paths[i])-1)
            # self.agent_names[name] = self.ax.text(d["start"][0], d["start"][1], name.replace('Robot', ''),fontsize=10)
            # self.agent_names[name].set_horizontalalignment('center')
            # self.agent_names[name].set_verticalalignment('center')
            # self.artists.append(self.agent_names[name])

        print("agents created!")

        # self.ax.set_axis_off()
        # self.fig.axes[0].set_visible(False)
        # self.fig.axes.get_yaxis().set_visible(False)

        # self.fig.tight_layout()
        self.init_func()
        print(int(self.T+1) * scale)
        self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                                            init_func=self.init_func,
                                            frames=int(self.T+1) * scale,
                                            interval=scale,
                                            blit=True, repeat=False)

        # FFwriter = animation.FFMpegWriter(fps=10)
        # self.anim.save('corner_dense_ddecbs.mp4',fps=10)
        self.anim.save('lak103_ddecbs.mp4',fps=10)
        # self.anim.save("exmaple_full.gif", fps=10, writer='pillow')

    def generate_obs(self):
        for i in range(1, self.map_sizeX, 3):
            for j in range(1, self.map_sizeY, 3):
                self.patches.append(
                    Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='black', edgecolor='red'))

    # def arrange_square_colors(self):
    #   self.colors=[]
    #   color_dict=dict()
    #   for d, i in zip(map["agents"], range(0, len(map["agents"]))):
    #       si=(d["start"][0],d["start"][1])
    #       gi=(d["goal"][0],d["goal"][1])
    #       if si not in color_dict:
    #           color_dict[si]=np.random.rand(3)
    #           color_dict[gi]=np.random.rand(3)
    #       self.colors.append(color_dict[si])

    # def arrange_block_colors(self):
    #      m=30
    #      dd=6
    #      self.colors=[]
    #      color_dict=dict()
    #      for d, i in zip(map["agents"], range(0, len(map["agents"]))):
    #          si=(d["start"][0],d["start"][1])
    #          s_id=(int(si[0]/dd),int(si[1]/dd))
    #          if s_id not in color_dict:
    #              color_dict[s_id]=np.random.rand(3)
    #          self.colors.append(color_dict[s_id])

    # def arrange_column_colors(self):

    #     self.colors=[]
    #     color_dict=dict()
    #     for d, i in zip(map["agents"], range(0, len(map["agents"]))):
    #         si=(d["start"][0],d["start"][1])
    #         gi=(d["goal"][0],d["goal"][1])
    #         g_id=int(gi[0]/3)
    #         if g_id not in color_dict:
    #             color_dict[g_id]=np.random.rand(3)
    #         self.colors.append(color_dict[g_id])

    def arrange_random_colors(self, m, n):
        # m,n=n,m

        self.colors = []

        for i in range(len(self.paths)):
            si = self.paths[i][0]
            gi = self.paths[i][-1]

            self.colors.append(color_for_goal(gi, m, n))

    def save(self, file_name, speed):
        self.anim.save(
            file_name,
            "ffmpeg",
            fps=10 * speed,
            dpi=800),
        # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

    def show(self):
        plt.show()

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def animate_func(self, i):
        for k in range(0, len(self.paths)):
            # agent = schedule["schedule"][agent_name]
            path = self.paths[k]
            # pos = self.getState(i / scale, agent)
            pos = self.getState(i / scale, path)
            p = (pos[0], pos[1])
            self.agents[k].center = p
            # self.agent_names[agent_name].set_position(p)

        # reset all colors

        return self.patches + self.artists

    def getState(self, t, d):
        idx = 0
        while idx < len(d) and idx < t:
            idx += 1
        if idx == 0:
            return np.array([float(d[0][0]), float(d[0][1])])
        elif idx < len(d):
            posLast = np.array([float(d[idx-1][0]), float(d[idx-1][1])])
            posNext = np.array([float(d[idx][0]), float(d[idx][1])])
        else:
            return np.array([float(d[-1][0]), float(d[-1][1])])
        # dt = d[idx]["t"] - d[idx-1]["t"]
        # t = (t - d[idx-1]["t"]) / dt
        t = t-(idx-1)
        pos = (posNext - posLast) * t + posLast
        return pos


def load_env(file_name):
    f = open(file_name)
    data_dict = json.load(f)
    paths = data_dict["paths"]
    xmax = data_dict["xmax"]
    ymax = data_dict["ymax"]
    obstacles = data_dict["obstacles"]
    return paths, xmax, ymax, obstacles


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map")
    # parser.add_argument("schedule", help="schedule for agents")
    parser.add_argument('--video', dest='video', default=None,
                        help="output video file (or leave empty to show on screen)")
    parser.add_argument("--speed", type=int, default=1, help="speedup-factor")
    args = parser.parse_args()

    # map_sizeX,map_sizeY,starts,goals=load_instance(args.map)
    paths, map_sizeX, map_sizeY, obstacles = load_env(args.map)
    map_sizeX,map_sizeY=map_sizeY,map_sizeX
    # paths,xmax,ymax,obstacles=load_env()

    # paths=load_schedule(args.schedule)
    # print(args.schedule)
    print("number of agents=", len(paths))
    animation = Animation(map_sizeX, map_sizeY, paths, obstacles)

    if args.video:
        animation.save(args.video, args.speed)
    else:
        animation.show()
