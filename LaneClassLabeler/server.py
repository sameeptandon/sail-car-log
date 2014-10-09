from flask import Flask, render_template
app = Flask(__name__)

@app.route('/')
def hello():
    return render_template('main.html', test='Hello')

if __name__ == '__main__':
    app.run()
