from flask import Flask, render_template
app = Flask(__name__)

@app.route('/')
def hello():
    return render_template('main.html', test='Hello',
                           video_src="static/test.jpg",
                           generated_src='static/test2.jpg')

if __name__ == '__main__':
    app.run(debug=True)
