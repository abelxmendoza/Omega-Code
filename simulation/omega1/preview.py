#!/usr/bin/env python3
"""Render the generated SDF's neutral pose for inspection (not physics)."""
from pathlib import Path
import math
import xml.etree.ElementTree as E
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parent

def transform(p):
    x,y,z,roll,pitch,yaw=map(float,p.split())
    cr,sr=math.cos(roll),math.sin(roll); cp,sp=math.cos(pitch),math.sin(pitch); cy,sy=math.cos(yaw),math.sin(yaw)
    r=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    return r,np.array([x,y,z])

def main():
    fig=plt.figure(figsize=(10,7),facecolor='#edf0f3'); ax=fig.add_subplot(projection='3d'); ax.set_facecolor('#edf0f3')
    all_faces=[]; all_colors=[]
    for link in E.parse(ROOT/'generated/models/omega1/model.sdf').findall('./model/link'):
        lr,lt=transform(link.findtext('pose'))
        for v in link.findall('visual'):
            vr,vt=transform(v.findtext('pose')); box=v.find('./geometry/box/size'); faces=[]
            if box is not None:
                dims=np.array(list(map(float,box.text.split())))/2
                vertices=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]])*dims
                for ids in [[0,1,3,2],[4,5,7,6],[0,1,5,4],[2,3,7,6],[0,2,6,4],[1,3,7,5]]: faces.append(vertices[ids])
            else:
                c=v.find('./geometry/cylinder'); r=float(c.findtext('radius')); h=float(c.findtext('length'))/2
                a=np.linspace(0,2*math.pi,33); ring=np.column_stack([r*np.cos(a),r*np.sin(a)])
                bottom=np.column_stack([ring,np.full(len(ring),-h)]); top=np.column_stack([ring,np.full(len(ring),h)])
                faces=[bottom,top]+[np.array([bottom[i],bottom[i+1],top[i+1],top[i]]) for i in range(32)]
            faces=[(f@vr.T+vt)@lr.T+lt for f in faces]
            color=list(map(float,v.findtext('./material/diffuse').split()))
            all_faces.extend(faces); all_colors.extend([color]*len(faces))
    ax.add_collection3d(Poly3DCollection(all_faces,facecolors=all_colors,edgecolors='#252835',linewidths=0.12))
    ax.set(xlim=(-0.15,0.15),ylim=(-0.13,0.13),zlim=(0,0.17),xlabel='X / forward (m)',ylabel='Y / left (m)',zlabel='Z (m)')
    ax.set_box_aspect((0.30,0.26,0.17)); ax.view_init(elev=25,azim=38)
    ax.set_title('OMEGA1 · first engineering model\nEstimated geometry — physical calibration pending',pad=15)
    fig.savefig(ROOT/'generated/preview.png',dpi=150,bbox_inches='tight'); plt.close(fig)
if __name__=='__main__': main()
