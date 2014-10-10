import cv2
import glob
import sys
from ArgParser import parse_args
from LaneClassLabeler import ImageLoader

from flask import Flask, render_template

app = Flask(__name__)

@app.route('/')
def hello():
    return render_template('main.html', test='Hello',
                           video_src="static/test.jpg",
                           generated_src='static/test2.jpg')

if __name__ == '__main__':
    args = parse_args('../process/data/4-20-14-280/280N_a', '280N_a2.avi')
    il = ImageLoader(args)
    I = il.nextFrame(10)
    cv2.imwrite('static/test.jpg', I)
    app.run(debug=True)
