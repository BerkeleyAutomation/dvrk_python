"""
Analyze pickle files.
"""
import os
import cv2
import sys
import pickle
import time
from os.path import join


# TODO: support analyzing multiple episodes together
def analyze_single(pth, episode_idx=None):
    target = 'tmp'
    with open(pth, 'r') as fh:
        data = pickle.load(fh)
    print('coverage: {}'.format(data['coverage']) )
    print('coverage len: {}'.format(len(data['coverage'])))
    print('c_img len:    {}'.format(len(data['c_img'])))
    print('d_img len:    {}'.format(len(data['d_img'])))
    print('actions len:  {}'.format(len(data['actions'])))
    for idx,(c_img,d_img) in enumerate(zip(data['c_img'], data['d_img'])):
        pth_c = join(target,'c_img_{}.png'.format(idx))
        pth_d = join(target,'d_img_{}.png'.format(idx))
        cv2.imwrite(pth_c, c_img)
        cv2.imwrite(pth_d, d_img)



if __name__ == "__main__":
    pth = join('results','tier1_color','ep_002.pkl')
    analyze_single(pth)
